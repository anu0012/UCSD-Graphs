import sys
import re
import os
import shutil
import commands

# git init
# git add .
# git commit -m "First commit"
# git remote add origin repository
# git push -u origin master

def main():
  list = ['git init','git add .','git commit -m "First commit"']
  for cmd in list:
    (status, output) = commands.getstatusoutput(cmd)
    if status:
      sys.stderr.write(output)
      sys.exit(1)
    print output
  repo = raw_input("Repository name please: ")
  list2 = ['git remote add origin '+repo,'git push -u origin master']
  for cmd in list2:
    (status, output) = commands.getstatusoutput(cmd)
    if status:
      sys.stderr.write(output)
      sys.exit(1)
    print output
    
if __name__ == "__main__":
  main()

  
