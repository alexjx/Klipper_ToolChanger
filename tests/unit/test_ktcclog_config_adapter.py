from ktcclog import KtccLogConfigCompatibility, load_config


class Config:
    def __init__(self, values=None):
        self.values = values or {}
        self.read = []

    def getint(self, name, default, **limits):
        self.read.append((name, limits))
        return int(self.values.get(name, default))


def test_adapter_consumes_all_deployed_options_without_runtime_registration():
    config = Config({"log_level": 2, "logfile_level": -1})
    adapter = load_config(config)

    assert isinstance(adapter, KtccLogConfigCompatibility)
    assert adapter.log_level == 2
    assert adapter.logfile_level == -1
    assert {name for name, _ in config.read} == {
        "log_level", "logfile_level", "log_statistics", "log_visual"
    }
