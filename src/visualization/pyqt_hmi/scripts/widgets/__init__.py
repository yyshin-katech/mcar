from .vehicle_view import VehicleViewWidget
from .status_indicator import StatusIndicator

# MainDisplayWindow pulls in ROS-only msg types; expose lazily to keep
# the rest of the package importable on hosts without the catkin workspace.
try:
    from .main_window import MainDisplayWindow  # noqa: F401
    __all__ = ['VehicleViewWidget', 'StatusIndicator', 'MainDisplayWindow']
except ImportError:
    __all__ = ['VehicleViewWidget', 'StatusIndicator']