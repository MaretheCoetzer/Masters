import sys
from pathlib import Path # if you haven't already done so

# Ensure that the root path (the StepUp package) is
# appended to the system path so our tests know where
# to look for the good stuff

file = Path(__file__).resolve()
parent, root = file.parent, file.parents[1]
print(f"root={root}")
sys.path.append(str(root))