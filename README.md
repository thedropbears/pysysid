# pysysid

 The Drop Bears' robot code for sysid in common mechanism types

## Setup

### Install dependencies

We use `uv` to manage our dependencies in our development environments.
This includes the Python version, and any Python packages such as `wpilib`.

Install `uv` by following the [`uv` docs](https://docs.astral.sh/uv/).

After installing `uv`, use it to create a virtual environment and install our dependencies.

```sh
uv sync
```

Then, download the roboRIO dependencies.

```sh
uv run python -m ensurepip
uv run robotpy --main <SYSID_PROJECT> sync --no-install
```

### pre-commit

[pre-commit][] is configured to run our formatters and linters.
These are enforced for all code committed to this project.

To use pre-commit, you must install it outside of this project's virtual environment.
Either use your system package manager, or use `uv tool`:

```sh
uv tool install pre-commit
```

You can then set up the pre-commit hooks to run on commit:

```sh
pre-commit install
```

## Run

### Simulation

``` bash
uv run robotpy --main <SYSID_PROJECT> sim
```

### Deploy to Robot

Once on robots network

``` bash
uv run robotpy --main <SYSID_PROJECT> deploy
```

### Test

``` bash
uv run robotpy --main <SYSID_PROJECT> test
```
