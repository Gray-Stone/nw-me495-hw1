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
        accumulated_error         : overall accumulated error between traveled distance in one lap, 
                                    and estimated straight line distance
    All members should be accessed through member functions. 
    All member functions uses an internal lock, Thus safe for multithreading usage (Ros multi ros callback groups).
    '''

    # This is required as soon as construction
    _straight_line_loop_length:float
    # accumulated distance traveled
    # current_loop_distance:float = 0
    _overall_distance: float = 0
    # This is updated per loop
    _completed_loops:int = 0
    _accumulated_error:float = 0

    _data_lock = threading.Lock()

    def get_current_loop_distance(self)->float:
        '''
        Get the 
        '''
        with self._data_lock:
            self._overall_distance - self._completed_loops* self._straight_line_loop_length - self._accumulated_error

    def add_traveled_distance(self , additional_distance:float)->None:
        '''
        Accumulate the newly traveled distance.
        '''
        with self._data_lock:
            self._overall_distance+=additional_distance

    def finished_another_loop(self)->None:
        '''
        Count the finished lap one more time, get the new lap's error and update the overall errors
        '''
        with self._data_lock
            current_loop_error = self.get_current_loop_distance() - self._straight_line_loop_length
            self._accumulated_error+=current_loop_error
            self._completed_loops+=1

    def generate_message(self)-> ErrorMetric:
        '''
        Generate error metric for publishing. 
        The content will be generated base on internal value of this class
        '''
        out = ErrorMetric()
        with self._data_lock:
            out.complete_loops = self._completed_loops
            out.actual_distance = self._overall_distance
            out.error = self._accumulated_error
