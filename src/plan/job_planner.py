from typing import List, Optional

from logzero import logger

from src.constant import CONSTANT
from src.floor import Coordinate, SectorMapSnapshot
from src.job import InstructionType, Job, JobInstruction
from src.operators import HT_Coordinate_View
from src.plan.job_tracker import JobTracker


class JobPlanner:
    """
    Coordinates job planning activities using HT tracker and sector map data.

    Attributes
    ----------
    ht_coord_tracker : HT_Coordinate_View
        An instance responsible for tracking the coordinates of HTs.
    sector_map_snapshot : SectorMapSnapshot
        A snapshot of the sector map representing the current state of the environment for planning.
    """

    def __init__(
        self,
        ht_coord_tracker: HT_Coordinate_View,
        sector_map_snapshot: SectorMapSnapshot,
    ):
        self.ht_coord_tracker = ht_coord_tracker
        self.sector_map_snapshot = sector_map_snapshot
        self.yard_usage = {}
        self.yard_last_assigned_tick = {}
        self.yard_ema_load = {}
        self.qc_last_assigned_tick = {}
        self.qc_ema_load = {}
        self.recent_ht_assignments = []
        self.current_tick = 0

    def is_deadlock(self):
        return self.ht_coord_tracker.is_deadlock()

    def get_non_moving_HT(self):
        return self.ht_coord_tracker.get_non_moving_HT()

    """ YOUR TASK HERE
    Objective: modify the following functions (including input arguments as you see fit) to achieve better planning efficiency.
        select_HT():
            select HT for the job based on your self-defined logic.
        select_yard():
            select yard for the job based on your self-defined logic.
        get_path_from_buffer_to_QC():
        get_path_from_buffer_to_yard():
        get_path_from_yard_to_buffer():
        get_path_from_QC_to_buffer():
            generate an efficient path for HT to navigate between listed locations (QC, yard, buffer).        
    """

    def plan(self, job_tracker: JobTracker) -> List[Job]:
        # logger.info("Planning started.")
        plannable_job_seqs = job_tracker.get_plannable_job_sequences()
        selected_HT_names = list()  # avoid selecting duplicated HT during the process
        new_jobs = list()  # container for newly created jobs

        # create job loop: ranging from 0 to at most 16 jobs
        for job_seq in plannable_job_seqs:
            # parse job info
            job = job_tracker.get_job(job_seq)
            job_info = job.get_job_info()
            job_type, QC_name, yard_name, alt_yard_names = [
                job_info[k]
                for k in ["job_type", "QC_name", "yard_name", "alt_yard_names"]
            ]

            # select HT for the job based on job type, return None if no HT available or applicable
            HT_name = self.select_HT(job_type, selected_HT_names, QC_name, yard_name)

            # not proceed with job planning if no available HTs
            if HT_name is None:
                break
            selected_HT_names.append(HT_name)

            # select yard if the job is DISCHARGE
            if job_type == CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE:
                yard_name = self.select_yard(yard_name)

            # record the assigned HT and yard
            job.assign_job(HT_name=HT_name, yard_name=yard_name)

            # construct the job instructions
            job_instructions = list()
            buffer_coord = self.ht_coord_tracker.get_coordinate(HT_name)

            # For DI job
            if job_type == CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE:

                # 1. Book QC resource
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.BOOK_QC,
                    )
                )

                # 2. HT drives from Buffer to QC[IN]
                buffer_coord = self.ht_coord_tracker.get_coordinate(HT_name)
                path = self.get_path_from_buffer_to_QC(buffer_coord, QC_name)
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                # 3. Work with QC
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.WORK_QC,
                        HT_name=HT_name,
                        QC_name=QC_name,
                    )
                )

                # 4. HT drives from QC to Buffer
                path = self.get_path_from_QC_to_buffer(QC_name, buffer_coord)
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                # 5. Book Yard resource
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.BOOK_YARD,
                    )
                )

                # 6. HT drives from Buffer to Yard[IN]
                path = self.get_path_from_buffer_to_yard(buffer_coord, yard_name)
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                # 7. Work with Yard
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.WORK_YARD,
                        HT_name=HT_name,
                        yard_name=yard_name,
                    )
                )

                # 8. HT drives from Yard to Buffer
                path = self.get_path_from_yard_to_buffer(yard_name, buffer_coord)
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

            # For LO job
            else:

                # 1. Book Yard resource
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.BOOK_YARD,
                    )
                )

                # 2. HT drives from buffer to Yard[IN]
                buffer_coord = self.ht_coord_tracker.get_coordinate(HT_name)
                path = self.get_path_from_buffer_to_yard(buffer_coord, yard_name)
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                # 3. Work with Yard
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.WORK_YARD,
                        HT_name=HT_name,
                        yard_name=yard_name,
                    )
                )

                # 4. HT drives from Yard to buffer
                path = self.get_path_from_yard_to_buffer(yard_name, buffer_coord)
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                # 5. Book QC resource
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.BOOK_QC,
                    )
                )

                # 6. HT drives from buffer to QC[IN]
                path = self.get_path_from_buffer_to_QC(buffer_coord, QC_name)
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                # 7. Work with QC
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.WORK_QC,
                        HT_name=HT_name,
                        QC_name=QC_name,
                    )
                )

                # 8. HT drives from QC to buffer
                path = self.get_path_from_QC_to_buffer(QC_name, buffer_coord)
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

            job.set_instructions(job_instructions)
            new_jobs.append(job)
            # logger.debug(f"{job}")

        return new_jobs

    def select_HT(self, job_type: str, selected_HT_names: List[str], QC_name: str = None, yard_name: str = None) -> str:
        """
        Selects an available HT (Horizontal Transport) based on the job type and a list of already selected HTs.

        For a discharge job, the method selects the first unselected HT from the left (start) of the buffer zone.
        For any other job type, it selects the first unselected HT from the right (end) of the buffer zone.

        Args:
            job_type (str): The type of job to be processed (e.g., discharge or other).
            selected_HT_names (List[str]): A list of HT names that are already selected or in use.

        Returns:
            str or None: The name of the selected HT if one is available; otherwise, None.
        """
        try: available_HTs = self.ht_coord_tracker.get_available_HTs()
        except: available_HTs = []
        candidates = [ht for ht in available_HTs if ht not in selected_HT_names]
        if not candidates: return None
        QC_IN_LANES = [2,4,7,9,12,14,17,19,22,24,27,29,32,34,37,39]
        QC_OUT_LANES = [3,5,8,10,13,15,18,20,23,25,28,30,33,35,38,40]
        def buf_to_qc_len(buf_x,qc_in_x): return abs(qc_in_x-buf_x)+2 if qc_in_x>=buf_x else buf_x+qc_in_x+4
        def qc_to_buf_len(buf_x,qc_out_x): return abs(buf_x-qc_out_x)+2 if qc_out_x<=buf_x else (42-buf_x)+8
        def buf_to_yard_len(buf_x,yard_in_x,yard_in_y):
            base=1
            if yard_in_x>buf_x: lanes_left=[x for x in QC_IN_LANES if x<buf_x];turn_x=max(lanes_left)if lanes_left else 1;return base+(buf_x-turn_x)+(yard_in_x-turn_x)+abs(yard_in_y-12)
            elif yard_in_x<buf_x: return base+(buf_x-yard_in_x)+abs(yard_in_y-7)
            else: return base+abs(yard_in_y-7)
        def yard_to_buf_len(buf_x,yard_out_x,yard_out_y):
            if buf_x<=yard_out_x: return (yard_out_x-buf_x)+abs(yard_out_y-7)
            else: lanes_right=[x for x in QC_OUT_LANES if x>buf_x];turn_x=min(lanes_right)if lanes_right else 41;return (turn_x-yard_out_x)+(turn_x-buf_x)+abs(yard_out_y-12)
        try: qc_sector=self.sector_map_snapshot.get_QC_sector(QC_name)if QC_name else None;qc_in=qc_sector.in_coord if qc_sector else None;qc_out=qc_sector.out_coord if qc_sector else None
        except: qc_in=qc_out=None
        try: yard_sector=self.sector_map_snapshot.get_yard_sector(yard_name)if yard_name else None;yard_in=yard_sector.in_coord if yard_sector else None;yard_out=yard_sector.out_coord if yard_sector else None
        except: yard_in=yard_out=None
        try: self.last_job_qc_name=QC_name if job_type==CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE else None;self.last_job_yard_name=yard_name if job_type!=CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE else None
        except: pass
        recent=getattr(self,"recent_ht_assignments",[])
        selected_xs=[self.ht_coord_tracker.get_coordinate(n).x for n in selected_HT_names if self.ht_coord_tracker.get_coordinate(n)]
        def recent_penalty(name): return max(0,5-(len(recent)-1-max((i for i,n in enumerate(recent)if n==name),default=len(recent))))*0.6 if name in recent else 0.0
        def spread_penalty(x): return sum((3-abs(x-sx))*0.4 for sx in selected_xs if abs(x-sx)<3)
        def position_bonus(buf_x):
            bonus=0
            if job_type==CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE and qc_in:
                if buf_x<=qc_in.x: bonus-=3
                if yard_in and abs(buf_x-qc_in.x)<abs(buf_x-yard_in.x): bonus-=2
            elif yard_in:
                if abs(buf_x-yard_in.x)<8: bonus-=3
                if qc_in and abs(buf_x-yard_in.x)<abs(buf_x-qc_in.x): bonus-=2
            return bonus
        # Heavily weight the immediate leg to destination; de-emphasize later legs
        # Micro-tuned around prior best (2.6/0.55)
        immediate_w,downstream_w=2.6,0.55
        best_ht,best_cost,best_coord=None,float("inf"),None
        for ht_name in candidates:
            coord=self.ht_coord_tracker.get_coordinate(ht_name)
            if not coord: continue
            buf_x=coord.x;total=0
            if job_type==CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE:
                # Immediate destination: QC in
                if qc_in: total+=buf_to_qc_len(buf_x,qc_in.x)*immediate_w
                # Subsequent legs
                if qc_out: total+=qc_to_buf_len(buf_x,qc_out.x)*downstream_w
                if yard_in: total+=buf_to_yard_len(buf_x,yard_in.x,yard_in.y)*downstream_w
                if yard_out: total+=yard_to_buf_len(buf_x,yard_out.x,yard_out.y)*downstream_w
            else:
                # Immediate destination: Yard in
                if yard_in: total+=buf_to_yard_len(buf_x,yard_in.x,yard_in.y)*immediate_w
                # Subsequent legs
                if yard_out: total+=yard_to_buf_len(buf_x,yard_out.x,yard_out.y)*downstream_w
                if qc_in: total+=buf_to_qc_len(buf_x,qc_in.x)*downstream_w
                if qc_out: total+=qc_to_buf_len(buf_x,qc_out.x)*downstream_w
            total+=recent_penalty(ht_name)+spread_penalty(buf_x)+position_bonus(buf_x)
            if total<best_cost: best_cost=total;best_ht=ht_name;best_coord=coord
        if best_ht: self.last_selected_ht_coord=best_coord;self.last_job_type=job_type;recent.append(best_ht);self.recent_ht_assignments=recent[-48:]
        return best_ht

    # YARD ASSIGNMENT LOGIC
    def select_yard(self, yard_name: str) -> str:
        """
        Choose a yard by balancing two goals:
        1) Even distribution across yards (fairness).
        2) Proximity to the currently selected HT waiting in the buffer.

        Lower score is better. We combine a simple distance-to-HT term with a
        fairness term that penalizes overused yards relative to the current minimum usage.
        """
        all_yards = CONSTANT.YARD_FLOOR.YARD_NAMES
        ht_coord = getattr(self, "last_selected_ht_coord", None) or Coordinate(21, 6)
        buffer_y = 6

        for yd in all_yards:
            if yd not in self.yard_usage:
                self.yard_usage[yd] = 0
            if yd not in self.yard_ema_load:
                self.yard_ema_load[yd] = 0.0

        min_usage = min((self.yard_usage.get(yd, 0) for yd in all_yards), default=0)


        dx_weight = 1.6        
        dy_weight = 0.8        
        fairness_weight = 3.92 
        hint_bonus = 0.8       
        repeat_penalty = 0.0   

        best_yard = None
        best_score = float("inf")

        yard_dist: dict[str, float] = {}
        yard_usage_now: dict[str, int] = {}
        min_dist = float("inf")
        for yd in all_yards:
            yard_sector = self.sector_map_snapshot.get_yard_sector(yd)
            yard_in = yard_sector.in_coord

            dx = abs(ht_coord.x - yard_in.x)
            dy = abs(buffer_y - yard_in.y)
            dist_score = dx * dx_weight + dy * dy_weight
            yard_dist[yd] = dist_score
            usage = self.yard_usage.get(yd, 0)
            yard_usage_now[yd] = usage
            if dist_score < min_dist:
                min_dist = dist_score

        for yd in all_yards:
            dist_score = yard_dist[yd]
            usage = yard_usage_now[yd]

            fairness_score = (usage - min_usage) * fairness_weight

            score = dist_score + fairness_score

            if yd == yard_name:
                score -= hint_bonus

            if yd == getattr(self, "last_assigned_yard", None):
                score += repeat_penalty

            if score < best_score:
                best_score = score
                best_yard = yd

        if best_yard is None:
            best_yard = yard_name

        ema_alpha = 0.90
        current_load = self.yard_usage[best_yard]
        self.yard_ema_load[best_yard] = (
            ema_alpha * self.yard_ema_load[best_yard]
            + (1 - ema_alpha) * current_load
        )
        self.yard_usage[best_yard] = current_load + 1
        self.yard_last_assigned_tick[best_yard] = self.current_tick
        self.last_assigned_yard = best_yard
        return best_yard

    # NAVIGATION LOGIC
    def get_path_from_buffer_to_QC(
        self, buffer_coord: Coordinate, QC_name: str
    ) -> List[Coordinate]:
        """
        Generates an efficient path from buffer to QC, utilizing a vertical shortcut 
        if the QC's X-coordinate is greater than or equal to the buffer's X-coordinate.
        Otherwise, it follows the original fixed loop to X=1.
        """
        QC_in_coord = self.sector_map_snapshot.get_QC_sector(QC_name).in_coord
        path = []
        
        # Current position
        curr_x, curr_y = buffer_coord.x, buffer_coord.y
        qc_travel_lane_y = 4
        qc_approach_lane_y = 5 

        # --- CONDITIONAL OPTIMIZATION START ---

        if QC_in_coord.x >= curr_x:
            # SCENARIO: QC is to the East (Right) or aligned. TAKE THE SHORTCUT!
            
            # 1. Move vertically from the buffer spot down/up to the Y=5 approach lane
            step_y = 1 if qc_approach_lane_y > curr_y else -1
            path.extend([Coordinate(curr_x, y) for y in range(curr_y + step_y, qc_approach_lane_y + step_y, step_y)])
            
            # Update current Y to 5
            if path:
                curr_x = path[-1].x
                curr_y = qc_approach_lane_y
            
            # 2. Travel EAST (Right) along the Y=5 lane until aligning with QC[IN].X
            step_x = 1 if QC_in_coord.x > curr_x else -1
            path.extend([Coordinate(x, curr_y) for x in range(curr_x + step_x, QC_in_coord.x + step_x, step_x)])
            
            # 3. Drop down vertically one step from Y=5 to Y=4 at QC[IN].X
            curr_x = path[-1].x
            curr_y = path[-1].y # Should be 5
            
            # Move vertically down to Y=4
            path.append(Coordinate(curr_x, qc_travel_lane_y))
        
        else:            
            # 1. Moves south to the highway left lane (y = 7).
            highway_lane_y = 7
            path = [Coordinate(buffer_coord.x, highway_lane_y)]

            # 2. Travels west along the highway to the left boundary (x = 1).
            path.extend(
                [Coordinate(x, highway_lane_y) for x in range(buffer_coord.x - 1, 0, -1)]
            )

            # 3. Moves north to the upper lane (y = 4).
            up_path_x = 1
            path.extend([Coordinate(up_path_x, y) for y in range(6, 3, -1)])
            
            # 4. Travels east to the IN coordinate of the specified QC.
            # Horizontal travel starts from X=2
            path.extend(
                [Coordinate(x, qc_travel_lane_y) for x in range(2, QC_in_coord.x + 1, 1)]
            )

        # --- CONDITIONAL OPTIMIZATION END ---
        
        # Ensure the final coordinate is the exact QC_in_coord
        if not path or path[-1] != QC_in_coord:
            path.append(QC_in_coord)
            
        return path

    def get_path_from_buffer_to_yard(
        self, buffer_coord: Coordinate, yard_name: str
    ) -> List[Coordinate]:

        yard_in_coord = self.sector_map_snapshot.get_yard_sector(yard_name).in_coord

        path: List[Coordinate] = []
        # Step 1: From Buffer (y=6) move down into Highway Left (y=7)
        if buffer_coord.y < 7:
            for y in range(buffer_coord.y + 1, 8):
                path.append(Coordinate(buffer_coord.x, y))
        elif buffer_coord.y > 7:
            for y in range(buffer_coord.y - 1, 6, -1):
                path.append(Coordinate(buffer_coord.x, y))

        curr_x = path[-1].x if path else buffer_coord.x
        target_x = yard_in_coord.x

        # Conditional Optimization Stratergy. 
        if target_x > curr_x:
            # --- NEW: Simplified path to go left and then down ---
            QC_IN_LANES = [2, 4, 7, 9, 12, 14, 17, 19, 22, 24, 27, 29, 32, 34, 37, 39]
            highway_lane_y = 7 # Define the y-level for leftward travel
            # 1. Find the closest available down-lane to the left            
            start_x = buffer_coord.x
            available_lanes = [x for x in QC_IN_LANES if x < start_x]
            turn_x = max(available_lanes) if available_lanes else 1

            # 2. Travels west along the highway to the chosen 'turn_x'
            path.extend(
                [Coordinate(x, highway_lane_y) for x in range(start_x - 1, turn_x - 1, -1)]
            )

            # 3. Travel down at "turn x"
            expressway_y = 12
            for y in range(highway_lane_y + 1, expressway_y + 1):
                path.append(Coordinate(turn_x, y))

            # 4. FIX: Travel east (right) from 'turn_x' to the target x-coordinate
            for x in range(turn_x + 1, target_x + 1):
                path.append(Coordinate(x, expressway_y))

            # 5. FIX: Add the final vertical move from the expressway to the yard
            current_y = expressway_y
            if current_y < yard_in_coord.y:
                for y in range(current_y + 1, yard_in_coord.y + 1):
                    path.append(Coordinate(target_x, y))
            elif current_y > yard_in_coord.y:
                for y in range(current_y - 1, yard_in_coord.y - 1, -1):
                    path.append(Coordinate(target_x, y))
        # -------------------------------------------------------------

        elif target_x < curr_x:
            # Need to move left: stay on Highway Left (y=7) and move left
            if not path or path[-1].y != 7:
                path.append(Coordinate(curr_x, 7))
            for x in range(curr_x - 1, target_x - 1, -1):
                path.append(Coordinate(x, 7))
            current_y = 7
            
            # Step 3: Go straight down to Yard IN at target_x
            if current_y < yard_in_coord.y:
                for y in range(current_y + 1, yard_in_coord.y + 1):
                    path.append(Coordinate(target_x, y))
            elif current_y > yard_in_coord.y:
                for y in range(current_y - 1, yard_in_coord.y - 1, -1):
                    path.append(Coordinate(target_x, y))

        else:
            # Already aligned on x
            current_y = path[-1].y if path else buffer_coord.y

            # Step 3: Go straight down to Yard IN at target_x
            if current_y < yard_in_coord.y:
                for y in range(current_y + 1, yard_in_coord.y + 1):
                    path.append(Coordinate(target_x, y))
            elif current_y > yard_in_coord.y:
                for y in range(current_y - 1, yard_in_coord.y - 1, -1):
                    path.append(Coordinate(target_x, y))

        if path[-1] != yard_in_coord:
            path.append(yard_in_coord)

        return path

    def get_path_from_yard_to_buffer(
        self, yard_name: str, buffer_coord: Coordinate
    ) -> List[Coordinate]:
        
        yard_out_coord = self.sector_map_snapshot.get_yard_sector(yard_name).out_coord

        # go to Yard[OUT] first
        path = [yard_out_coord]
        target_x = buffer_coord.x
        curr_y = buffer_coord.y

        if buffer_coord.x <= yard_out_coord.x: 
            # SCENARIO 1 : Buffer to the left of our yard. Direct Path.
            highway_y = 7 
            # Enter Y = 7 by moving upwards. 
            for y in range(yard_out_coord.y - 1, highway_y - 1, -1):
                path.append(Coordinate(yard_out_coord.x, y))

            # Moves in the left lane. 
            path.extend(
                [Coordinate(x, highway_y) for x in range(yard_out_coord.x - 1, buffer_coord.x - 1, -1)]
            )

            # Final buffer coordinate will be added at the end to avoid duplicates

        else:  # Buffer is to the right; take expressway (y=12) to a suitable up-lane, then back along y=7
            QC_OUT_LANES = [3, 5, 8, 10, 13, 15, 18, 20, 23, 25, 28, 30, 33, 35, 38, 40]
            highway_lane_y = 12
            # Choose the first up-lane to the right of the buffer to avoid conflicts
            start_x = buffer_coord.x
            available_lanes = [x for x in QC_OUT_LANES if x > start_x]
            turn_x = min(available_lanes) if available_lanes else 41

            # Travel east on y=12 from yard_out to turn_x
            path.extend([Coordinate(x, highway_lane_y) for x in range(yard_out_coord.x, turn_x + 1, 1)])

            # Move up to Highway Left (y=7) via the up-lane at turn_x
            up_path_x = turn_x
            path.extend([Coordinate(up_path_x, y) for y in range(11, 6, -1)])

            # Travel west on y=7 back to the buffer x
            highway_lane_y = 7
            path.extend([Coordinate(x, highway_lane_y) for x in range(turn_x - 1, buffer_coord.x - 1, -1)])

        # Add final buffer coordinate
        path.append(buffer_coord)

        return path    

    def get_path_from_QC_to_buffer(
        self, QC_name: str, buffer_coord: Coordinate
    ) -> List[Coordinate]:
        """
        Generates a path from a Quay Crane (QC) OUT coordinate to a buffer location.

        The path follows this route:
        1. Starts at the QC OUT coordinate.
        2. Moves south to the QC travel lane (y = 4).
        3. Travels east along the QC travel lane to the right boundary.
        4. Moves south to the Highway Left lane (y = 7).
        5. Travels west along the highway left lane to the buffer coordinate.

        Args:
            QC_name (str): The name of the Quay Crane from which the path starts.
            buffer_coord (Coordinate): The destination coordinate in the buffer zone.

        Returns:
            List[Coordinate]: A list of coordinates representing the path from the QC to the buffer.
        """
        QC_out_coord = self.sector_map_snapshot.get_QC_sector(QC_name).out_coord
        QC_out_x = QC_out_coord.x
        buffer_x = buffer_coord.x

        # Initialize the path with the starting QC coordinate
        path = [QC_out_coord]
        curr_x = QC_out_x
        curr_y = QC_out_coord.y

        # Main logic to determine the path based on the QC and buffer X-coordinates
        if QC_out_x <= buffer_x:
            # --- Path Generation for Case 1: Use QC Travel Lane Y=5 ---
            qc_travel_lane_y = 5
            
            # 1. Move South to the QC Travel Lane (y=5)
            step_y = 1 # Assuming QC_out_coord.y is always less than 5
            path.extend([Coordinate(curr_x, y) for y in range(curr_y + step_y, qc_travel_lane_y + step_y, step_y)])
            
            # 2. Travel East along the lane until x-coordinate matches buffer_x
            path.extend(
                [Coordinate(x, qc_travel_lane_y) for x in range(curr_x + 1, buffer_x + 1, 1)]
            )
            
            # 3. Move vertically from the travel lane to the buffer's y-coordinate
            # Update current position after the horizontal move
            curr_y_after_horizontal = qc_travel_lane_y
            dest_y = buffer_coord.y
            
            # Determine vertical direction (down or up)
            step_y_final = 1 if dest_y > curr_y_after_horizontal else -1
            
            path.extend([
                Coordinate(buffer_x, y) for y in range(curr_y_after_horizontal + step_y_final, dest_y, step_y_final)
            ])

        else:
            # --- Path Generation for Case 2: Use QC Travel Lane Y=4 ---
            qc_travel_lane_y = 4
            
            # 1. Move South to the QC Travel Lane (y=4)
            step_y = 1 # Assuming QC_out_coord.y is always less than 4
            path.extend([Coordinate(curr_x, y) for y in range(curr_y + step_y, qc_travel_lane_y + step_y, step_y)])

            # 2. Travel East along the lane to the right boundary (x=42)
            path.extend(
                [Coordinate(x, qc_travel_lane_y) for x in range(QC_out_x + 1, 43, 1)]
            )

            # 3. Move South to the Highway Left Lane (y=7)
            down_path_x = 42
            path.extend([Coordinate(down_path_x, y) for y in range(qc_travel_lane_y + 1, 8, 1)])

            # 4. Travel West along the highway to the buffer's x-coordinate
            highway_lane_y = 7
            path.extend(
                [Coordinate(x, highway_lane_y) for x in range(41, buffer_x - 1, -1)]
            )
        
        # Finally, add the specific buffer coordinate to complete the path
        path.append(buffer_coord)

        return path