"""pytest setup for tools/bench: registers the `stack` marker (like llm's `llm` marker)."""


def pytest_configure(config):
    config.addinivalue_line(
        'markers',
        'stack: needs the LIVE stack (mission services, llama-server on 8083) and '
        'F1TENTH_CAR_SAFE=1 because it starts real missions; skipped otherwise')
