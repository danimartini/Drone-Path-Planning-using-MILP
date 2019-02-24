# Author: Daniel Martini Jimenez
# Date: 23-2-2019
def write_text(px,py,pz,tot_time):
    file=open('coordinates.txt','w')
    file.write('The total path takes %s\n'%(tot_time))
    for n in range(len(px)):
        file.write('%s \t %s \t %s\n'%(px[n].X,py[n].X,pz[n].X))

    file.close()
    return


