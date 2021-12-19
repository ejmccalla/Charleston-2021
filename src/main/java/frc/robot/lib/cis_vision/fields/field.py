from fields.field_elements_2019 import field_elements_list_in
from fields.field_image_and_offsets_2019 import field_img_bgr, field_img_offsets_in
# from fields.field_elements_shed import field_elements_list_in
# from fields.field_image_and_offsets_shed import field_img_bgr, field_img_offsets_in
from dataclasses import dataclass
from typing import List
import numpy as np

@dataclass
class Field:
    '''A dataclass of the field elements.

    Attributes:
        elements_in (:obj:`list` of :obj:`list` of :obj:`tuple(float,float)`):
            A list of the field elements where the first element is relative to
            the field coordate system and subsequent elements are relative to
            the preceding element.
        field_img_offsets_in (:obj:`list` of :obj:`tuple(float,float)`): A list
            of two tuples defining the bounding box of the playing field within
            img_bgr.
        img_bgr (:obj:`numpy.ndarray`): An image of the field. This can be
            found in the game documents. 
        width_ft (`float`): The width of the field (y-direction).
        length_ft (`float`): The length of the field (x-direction).
    '''
    elements_in: List[float]
    img_offsets_in: List[float]
    img_bgr: np.ndarray
    width_ft: float
    length_ft: float

field = Field( field_elements_list_in, field_img_offsets_in, field_img_bgr, 27.0, 54.0 )
# field = Field( field_elements_list_in, field_img_offsets_in, field_img_bgr, 17.5, 24.0 )



