from collections import defaultdict
from typing import Dict, List, Optional, Tuple

from logzero import logger

from src.constant import CONSTANT
from src.floor import Coordinate, SectorMapSnapshot
from src.job import InstructionType, Job, JobInstruction, Status
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
            HT_name = self.select_HT(job_type, selected_HT_names)

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

    # HT ASSIGNMENT LOGIC
    def select_HT(self, job_type: str, selected_HT_names: List[str]) -> str:
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
        plannable_HTs = self.ht_coord_tracker.get_available_HTs()
        selected_HT = None
        # if DI job, pick the HT on far left of buffer zone
        if job_type == CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE:
            for HT_name in plannable_HTs:
                if HT_name not in selected_HT_names:
                    selected_HT = HT_name
                    break
        # otherwise far right
        else:
            for HT_name in plannable_HTs[::-1]:
                if HT_name not in selected_HT_names:
                    selected_HT = HT_name
                    break
        return selected_HT

    def select_yard(self, yard_name: str) -> str:
        """
        Selects a yard for use. Currently, simply returns the provided yard name.

        Args:
            yard_name (str): The name of the yard to select.

        Returns:
            str: The selected yard name.
        """
        return yard_name

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
            # SCENARIO: QC is to the West (Left) and requires the original down-loop.
            
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

        # Changed the Go Left or right, now we do all left. 
        if target_x > curr_x:
            # --- NEW: Simplified path to go left and then down ---

            # 1. Travel LEFT along y=7 until you reach x=1
            go_left_y = 7
            go_down_x = 1
            for x in range(curr_x - 1, go_down_x - 1, -1):
                path.append(Coordinate(x, go_left_y))

            # 2. (Assuming it goes down to y=12 based on previous logic)
            end_y = 12
            for y in range(go_left_y + 1, end_y + 1):
                path.append(Coordinate(go_down_x, y)) 
            
            # Travel Right Along this expressway and return to x_coor of Yard In.
            highway_lane_y = 12

            path.extend(
                [Coordinate(x, highway_lane_y) for x in range(2, yard_in_coord.x + 1, 1)]
            )
            path.append(yard_in_coord)

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
        
        """
        Generates a path from a yard OUT area's coordinate to a buffer location.

        The path follows this route:
        1. Starts at the yard OUT coordinate.
        2. Moves east along the highway lane (y = 12) towards the second-to-right boundary.
        3. Moves north to the Highway Left lane (y = 7).
        4. Travels west along the highway left lane to the target buffer coordinate.

        Args:
            yard_name (str): The name of the yard from which the path starts.
            buffer_coord (Coordinate): The destination coordinate in the buffer zone.

        Returns:
            List[Coordinate]: A list of coordinates representing the path from the yard to the buffer.
        """
        yard_out_coord = self.sector_map_snapshot.get_yard_sector(yard_name).out_coord

        # go to Yard[OUT] first
        path = [yard_out_coord]

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

        else : 

            # enter highway lane, go to tile second-to-right boundary
            highway_lane_y = 12

            path.extend(
                [Coordinate(x, highway_lane_y) for x in range(yard_out_coord.x, 42, 1)]
            )

            # go to Highway Left lane (7)
            up_path_x = 41
            path.extend([Coordinate(up_path_x, y) for y in range(11, 6, -1)])

            # navigate back to original buffer
            highway_lane_y = 7
            path.extend(
                [Coordinate(x, highway_lane_y) for x in range(40, buffer_coord.x - 1, -1)]
            )

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
