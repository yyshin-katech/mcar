try:
    from .shapefile_loader import load_shapefile  # noqa: F401
    __all__ = ['load_shapefile']
except ImportError:
    # PyShp not installed on this host — map loader will be unavailable,
    # but the rest of the package (theme, hmi_state) still imports cleanly.
    __all__ = []