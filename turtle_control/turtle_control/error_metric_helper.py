from turtle_interface.msg import ErrorMetric
from dataclasses import dataclass
import threading


@dataclass
class ErrorMeasurer():
    '''
    The data class help collect and compute the error metric of controlling turtlesim.
    Members:
        straight_line_loop_length : set on construction 
        overall_distance          : accumulated overall travel distance.
        completed_loops           : completed loops. 

    All members should be accessed through member functions. 
    All member functions uses an internal lock, Thus safe for multithreading usage (Ros multi ros callback groups).
    '''

    # This is required as soon as construction
    straight_line_loop_length: float
    # accumulated distance traveled
    overall_distance: float = 0.0
    # This is updated per loop
    completed_loops: int = 0

    # Internal lock shared by all member functions. Must be a RLock since they reference each other.
    _data_lock = threading.RLock()

    def get_current_loop_distance(self) -> float:
        '''
        Get the distance traveled within this loop
        '''
        with self._data_lock:
            return self.overall_distance - self.completed_loops * self.straight_line_loop_length - self.get_accumulated_error(
            )

    def add_traveled_distance(self, additional_distance: float) -> None:
        '''
        Accumulate the newly traveled distance.
        '''
        with self._data_lock:
            self.overall_distance += additional_distance

    def finished_another_loop(self) -> None:
        '''
        Count the finished lap one more time, get the new lap's error and update the overall errors
        '''

        with self._data_lock:
            self.completed_loops += 1

    def get_accumulated_error(self) -> float:
        '''
        Calculate accumulated error base on current distance and laps completed 
        '''
        with self._data_lock:
            return self.overall_distance - self.straight_line_loop_length * self.completed_loops

    def generate_error_metric_message(self) -> ErrorMetric:
        '''
        Generate error metric for publishing. 
        The content will be generated base on internal value of this class
        '''
        out = ErrorMetric()
        with self._data_lock:
            out.complete_loops = self.completed_loops
            out.actual_distance = self.overall_distance
            out.error = self.get_accumulated_error()
        return out

    def __str__(self) -> str:
        """String-ify the class for printing in debug statement.
        Will print more then just the slots. some derived values are included for better debugging. 

        Returns:
            str: class info in string
        """
        return f"Traveled: {self.overall_distance}; loops {self.completed_loops}; estimated_overall: {self.completed_loops * self.straight_line_loop_length}; Error: {self.get_accumulated_error()}"
