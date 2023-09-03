# StepUp

Welcome to the StepUp application, also known as KateBush.

## Setup

All thats needed to configure ones workspace to run KateBush is run the setup script from
the configuration directory:

```bash
config/setup.sh
```

## Jupytr conversion

We can convert the jupytr notebooks into python scripts!

Get jupytext: https://jupytext.readthedocs.io/en/latest/install.html
```
pip install jupytext --upgrade
```

And then run the conversion with that:https://jupytext.readthedocs.io/en/latest/using-cli.html
```
jupytext --to py *.ipynb
```

## Virtual Box Stuff

Virtual box is setup with Pardiso solver as the default ipopt solver!

## Run unit tests
```
cd ./test
python3 -m unittest
```