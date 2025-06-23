import math
import structlog
import dataclasses

@dataclasses.dataclass
class RobotLocation:
    x: float
    y: float
    theta: float
    
class Robot:

    def __init__(self, logger: structlog.stdlib.BoundLogger):

        # register log handler
        self.logger = logger

        # register location
        self._location = RobotLocation(x=0.0, y=0.0, theta=0.0)

        self.logger.info("[Robot] Finished initializing Robot")
    
    def get_location(self) -> RobotLocation:
        return self._location
    
    def move(self, x: float, y: float, theta: float):

        # Step 1: Rotate to the correct angle (robot and goal should be aligned)
        goal_vec_x = x - self._location.x
        goal_vec_y = y - self._location.y

        
