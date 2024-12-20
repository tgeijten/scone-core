from sconetools import sconepy
import io
import sys

old_stdout = sys.stdout
new_stdout = io.StringIO()
sys.stdout = new_stdout
help(sconepy)
sys.stdout = old_stdout
help_output = new_stdout.getvalue()

with open("../sconepy/sconepy_help.txt", "w") as f:
	f.write(help_output)