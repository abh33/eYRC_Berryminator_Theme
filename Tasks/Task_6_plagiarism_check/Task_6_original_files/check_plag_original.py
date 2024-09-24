import mosspy
import glob
import os

userid=67952897

m = mosspy.Moss(userid, "python")
filename = "theme_implementation.py"
m.addBaseFile(filename)
print(m)

files = glob.glob("*theme_implementation_primary.py")

for team_file in files:
    print(team_file)
    m.addFile(team_file)

url = m.send(lambda file_path, display_name: print('*', end='', flush=True))
print()

print ("Report Url: " + url)