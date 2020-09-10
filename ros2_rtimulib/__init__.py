from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

def better_get_parameter_or(self: Node,
    name: str,
    alternative_value: Optional[Parameter]=None) -> Parameter:
    """fix the method that is usually on a Node so that it always returns a parameter Type
    
    Arguments:
        self {Node} -- [description]
        name {str} -- [description]
    
    Keyword Arguments:
        alternative_value {Optional[Parameter]} -- a default value \
            if the parameter is no preset (default: {None})
    
    Returns:
        Parameter -- the desired parameter
    """
    if alternative_value is None:
        alternative_value = Parameter(name, Parameter.Type.NOT_SET)

    res = self._parameters.get(name, alternative_value)
    if not isinstance(res, Parameter):
        res = Parameter(name, value=res)
    return res
