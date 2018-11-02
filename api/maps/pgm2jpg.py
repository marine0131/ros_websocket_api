from PIL import Image
import os.path
import glob
import re
def pgm2png(pgm_file, png_dir):
    pgm = Image.open(pgm_file)
    name = (str)(os.path.join(png_dir, os.path.splitext(os.path.basename(pgm_file))[0]))+".png"
    pgm.save(name)
    os.remove(pgm_file)

#for pgm_file in glob.glob("./company_map0616_new.pgm"):
#    pgm2png(pgm_file, "./")
lines = open("company_map.yaml").readlines()
lines[0] = re.sub(r"pgm","png",lines[0])
f = open("company_map.yaml","w+").writelines(lines)
