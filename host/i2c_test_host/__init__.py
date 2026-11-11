from .peers import DutDevice, load_peers

__all__ = [
	"BringupConfig",
	"BringupError",
	"BringupStack",
	"DutDevice",
	"load_bringup_config",
	"load_peers",
]


def __getattr__(name: str):
	if name in {"BringupConfig", "BringupError", "BringupStack", "load_bringup_config"}:
		from .bringup import (
			BringupConfig,
			BringupError,
			BringupStack,
			load_bringup_config,
		)

		exports = {
			"BringupConfig": BringupConfig,
			"BringupError": BringupError,
			"BringupStack": BringupStack,
			"load_bringup_config": load_bringup_config,
		}
		return exports[name]
	raise AttributeError(f"module {__name__!r} has no attribute {name!r}")