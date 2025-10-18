from typing import List
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
        self.yard_loads = {} 

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
            HT_name = self.select_HT(job, selected_HT_names)
            
            # not proceed with job planning if no available HTs
            if HT_name is None:
                break
            selected_HT_names.append(HT_name)

            # select yard if the job is DISCHARGE
            if job_type == CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE:
                yard_name = self.select_yard(yard_name, alt_yard_names, QC_name)

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
    def select_HT(self, job, selected_HT_names):
        """
        Selects the closest available HT. Prioritizes HTs that are currently 
        non-moving/stationary to maximize immediate assignment and assignment rate.
        """
        best_ht = None
        shortest_distance = float('inf')

        # 1. Get the job's starting location (Original logic - no change)
        job_info = job.get_job_info()
        if job_info['job_type'] == CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE:
            start_location = self.sector_map_snapshot.get_QC_sector(job_info['QC_name']).in_coord
        else:
            start_location = self.sector_map_snapshot.get_yard_sector(job_info['yard_name']).in_coord

        # --- ENHANCED LOGIC BLOCK (FIXED) ---
        
        # 2. Get the pool of all available HTs not yet assigned in this cycle
        all_eligible_HTs = [
            ht for ht in self.ht_coord_tracker.get_available_HTs() 
            if ht not in selected_HT_names
        ]
        
        # 3. Safely handle the return value of get_non_moving_HT()
        try:
            # Attempt to convert to a set of strings for efficient searching
            non_moving_HTs = set(self.ht_coord_tracker.get_non_moving_HT())
        except TypeError:
            # If a TypeError occurs (e.g., trying to iterate over an integer), 
            # assume the return value was invalid/empty for our purposes.
            non_moving_HTs = set()
        
        # 4. Create the prioritized search pool (only non-moving eligible HTs)
        # This line is now safe because non_moving_HTs is guaranteed to be a set.
        prioritized_pool = [ht for ht in all_eligible_HTs if ht in non_moving_HTs]
        
        # 5. Determine the final pool: Non-moving first, otherwise all eligible HTs
        HT_pool_to_search = prioritized_pool if prioritized_pool else all_eligible_HTs
        
        # 6. Find the closest HT from the determined pool
        for ht_name in HT_pool_to_search:
            ht_coord = self.ht_coord_tracker.get_coordinate(ht_name)
            # Use simple Manhattan distance for speed
            distance = abs(ht_coord.x - start_location.x) + abs(ht_coord.y - start_location.y)

            if distance < shortest_distance:
                shortest_distance = distance
                best_ht = ht_name
                
        return best_ht

    # YARD ASSIGNMENT LOGIC
    def select_yard(self, default_yard, alt_yards, qc_name): 
        """
        Selects the best yard by balancing travel distance (from QC to Yard)
        and congestion load (preventing slowness at busy yards).
        """
        best_yard = default_yard
        lowest_cost = float('inf')

        # Combine the default and alternative yards into one list to check
        possible_yards = [default_yard] + alt_yards
        
        # Define cost parameters
        YARD_CAPACITY = 700 
        CONGESTION_WEIGHT = 10 
        
        for yard_name in possible_yards:
            # Get the current number of jobs assigned to this yard
            current_load = self.yard_loads.get(yard_name, 0)
            
            # Disqualify the yard if it's at or over the capacity limit
            if current_load >= YARD_CAPACITY:
                continue

            # Use simple Manhattan distance to estimate travel cost
            qc_coord = self.sector_map_snapshot.get_QC_sector(qc_name).in_coord
            yard_coord = self.sector_map_snapshot.get_yard_sector(yard_name).in_coord
            distance = abs(qc_coord.x - yard_coord.x) + abs(qc_coord.y - yard_coord.y)
            
            # The cost is a combination of distance and a penalty for being full
            cost = distance + (current_load * CONGESTION_WEIGHT) 
            
            if cost < lowest_cost:
                lowest_cost = cost
                best_yard = yard_name
                
        # If a best yard was found, update its load count and return it
        if best_yard:
            self.yard_loads[best_yard] = self.yard_loads.get(best_yard, 0) + 1
            return best_yard
        
        # As a fallback, return the default yard
        return default_yard

    # NAVIGATION LOGIC
    def get_path_from_buffer_to_QC(
        self, buffer_coord: Coordinate, QC_name: str
    ) -> List[Coordinate]:
        """
        Generates a path from a buffer location to a Quay Crane (QC) input coordinate.

        The path follows a predefined route:
        1. Moves south to the highway left lane (y = 7).
        2. Travels west along the highway to the left boundary (x = 1).
        3. Moves north to the upper lane (y = 4).
        4. Travels east to the IN coordinate of the specified QC.

        Args:
            buffer_coord (Coordinate): The starting coordinate in the buffer zone.
            QC_name (str): The name of the Quay Crane to which the path should lead.

        Returns:
            List[Coordinate]: A list of coordinates representing the path from the buffer to the QC.
        """
        QC_in_coord = self.sector_map_snapshot.get_QC_sector(QC_name).in_coord

        # go South to take Highway Left lane (y=7)
        highway_lane_y = 7
        path = [Coordinate(buffer_coord.x, highway_lane_y)]

        # then go to the left boundary
        path.extend(
            [Coordinate(x, highway_lane_y) for x in range(buffer_coord.x - 1, 0, -1)]
        )

        # then go to upper boundary and navigate to QC_in
        up_path_x = 1
        path.extend([Coordinate(up_path_x, y) for y in range(6, 3, -1)])
        qc_travel_lane_y = 4
        path.extend(
            [Coordinate(x, qc_travel_lane_y) for x in range(2, QC_in_coord.x + 1, 1)]
        )
        path.append(QC_in_coord)
        return path

    def get_path_from_buffer_to_yard(
        self, buffer_coord: Coordinate, yard_name: str
    ) -> List[Coordinate]:
        """
        Generates a path from a buffer location to a yard IN area's coordinate.

        The path follows a specific route:
        1. Moves north to the QC travel lane (y = 5).
        2. Travels east to the right boundary of the sector (x = 42).
        3. Moves south to the Highway Left lane (y = 11).
        4. Travels west along the highway to the left boundary (x = 1).
        5. Moves south to the lower boundary (y = 12).
        6. Travels east to the IN coordinate of the specified yard.

        Args:
            buffer_coord (Coordinate): The starting coordinate in the buffer zone.
            yard_name (str): The name of the yard to which the path should lead.

        Returns:
            List[Coordinate]: A list of coordinates representing the path from the buffer to the yard.
        """
        yard_in_coord = self.sector_map_snapshot.get_yard_sector(yard_name).in_coord

        # Go North to take QC travel lane (y=5), then go to right boundary
        path = [Coordinate(buffer_coord.x, buffer_coord.y - 1)]
        qc_lane_y = 5
        path.extend(
            [Coordinate(x, qc_lane_y) for x in range(buffer_coord.x + 1, 43, 1)]
        )

        # go down to Highway Left lane(11), then takes left most
        down_path_x = 42
        path.extend([Coordinate(down_path_x, y) for y in range(6, 12, 1)])
        highway_lane_y = 11
        path.extend([Coordinate(x, highway_lane_y) for x in range(41, 0, -1)])

        # go to lower boundary, then navigate to Yard[IN]
        highway_lane_y = 12
        path.append(Coordinate(1, highway_lane_y))
        path.extend(
            [Coordinate(x, highway_lane_y) for x in range(2, yard_in_coord.x + 1, 1)]
        )
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

        # go to QC_out first
        path = [QC_out_coord]

        # go South to take QC Travel Lane all the way to right boundary
        qc_travel_lane_y = 4
        path.append(Coordinate(QC_out_coord.x, qc_travel_lane_y))
        path.extend(
            [Coordinate(x, qc_travel_lane_y) for x in range(QC_out_coord.x + 1, 43, 1)]
        )

        # go down to Highway Left lane(7), then takes left most
        down_path_x = 42
        path.extend([Coordinate(down_path_x, y) for y in range(5, 8, 1)])

        # navigate back to buffer
        highway_lane_y = 7
        path.extend(
            [Coordinate(x, highway_lane_y) for x in range(41, buffer_coord.x - 1, -1)]
        )
        path.append(buffer_coord)

        return path




