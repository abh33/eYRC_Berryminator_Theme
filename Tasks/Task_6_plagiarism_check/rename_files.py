import os
import glob
import shutil

src_1 = "Task_6_original_files"
src_2 = "Task_6_bonus_files"

dest = "Task_6_all_files"

filenames_original = glob.glob(src_1 + "\\" + "*theme_implementation_primary.py")
print(filenames_original)
filenames_bonus = glob.glob(src_2 + "\\" + "*theme_implementation_primary.py")
print(filenames_bonus)


for i in filenames_original:
    print(i)
    index1 = i.find("\\")
    dest_filename = dest + "\\original_" + i[index1+1:]
    print(dest_filename)
    os.rename(i, dest_filename)

for i in filenames_bonus:
    print(i)
    index1 = i.find("\\")
    dest_filename = dest + "\\bonus_" + i[index1+1:]
    print(dest_filename)
    os.rename(i, dest_filename)