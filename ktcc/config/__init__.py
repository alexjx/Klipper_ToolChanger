"""Order-independent configuration collection and resolution."""

from .normalization import normalize_tool_group_section, normalize_tool_section
from .resolver import ConfigBuilder, ResourceCatalog, resolve_config
from .schema import ConfigError, RawTool, RawToolGroup

__all__ = [name for name in globals() if not name.startswith("_")]
