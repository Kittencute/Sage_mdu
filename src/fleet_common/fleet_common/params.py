from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.parameter import Parameter

NUMERIC = (Parameter.Type.DOUBLE, Parameter.Type.INTEGER)


def declare(node, name, kind, description, low=None, high=None, constraints=""):
    if (low is None) != (high is None):
        raise ValueError(f"declare parameter {name} failed: cause: a range needs both low and high")
    if kind in NUMERIC and low is None and not constraints:
        raise ValueError(
            f"declare parameter {name} failed: cause: a numeric parameter needs a range or constraints"
        )
    descriptor = ParameterDescriptor(
        description=description, additional_constraints=constraints, read_only=True
    )
    if low is not None:
        if kind is Parameter.Type.DOUBLE:
            descriptor.floating_point_range = [
                FloatingPointRange(from_value=float(low), to_value=float(high), step=0.0)
            ]
        elif kind is Parameter.Type.INTEGER:
            descriptor.integer_range = [
                IntegerRange(from_value=int(low), to_value=int(high), step=0)
            ]
        else:
            raise ValueError(f"declare parameter {name} failed: cause: {kind.name} takes no range")
    value = node.declare_parameter(name, kind, descriptor).value
    if value is None:
        raise RuntimeError(f"read parameter {name} failed: cause: no value was given")
    return value
