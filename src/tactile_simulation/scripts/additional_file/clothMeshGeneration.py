


############ Parameters #############

TO_WRITE_FILE_NAME = "myCloth"

NUM_MARKER_POINT = 7
NUM_VERTICES_BETWEEN_MARKER = 1
MAX_X_IN_M = 1
MIN_X_IN_M = -1
MAX_Y_IN_M = 1
MIN_Y_IN_M = -1

NUM_VERTICES = 2+NUM_MARKER_POINT+(NUM_MARKER_POINT+1)*NUM_VERTICES_BETWEEN_MARKER ## edge + marker_point + (marker_point+1)*vertices_between_marker

def writeLine(string):
    f.write(string)
    f.write("\n")
    return    


vertices = []
verticesXY = []
faces = []

f = open(TO_WRITE_FILE_NAME+".obj","w")
writeLine("mtllib "+TO_WRITE_FILE_NAME+".mtl")


for i in range(NUM_VERTICES):
    for j in range(NUM_VERTICES):
        xi = round(MIN_X_IN_M+i*(MAX_X_IN_M-MIN_X_IN_M)/(NUM_VERTICES-1),5)
        yi = round(MIN_Y_IN_M+j*(MAX_Y_IN_M-MIN_Y_IN_M)/(NUM_VERTICES-1),5)
        vertices.append((i*NUM_VERTICES+j+1,(xi,yi,0)))




for i in range(NUM_VERTICES):    
    verticesXY.append(vertices[i*NUM_VERTICES:(i+1)*NUM_VERTICES])

# visualize verticesXY
for i in range(NUM_VERTICES):
    temp = []
    for j in range(NUM_VERTICES):
        temp.append(verticesXY[i][j][0])
        
    print(temp)
    

for vertex in vertices:
    x = vertex[1][0]
    y = vertex[1][1]
    z = vertex[1][2]
    writeLine("v "+ str(x) + " " + str(y) + " " + str(z))
    print(vertex)

writeLine("\n")

writeLine("vn 0.0 0.0 1.0")

writeLine("\n")

for vertex in vertices:
    x = vertex[1][0]
    y = vertex[1][1]
    z = vertex[1][2]
    xt = round((x-MIN_X_IN_M)/(MAX_X_IN_M-MIN_X_IN_M),5)
    yt = round((y-MIN_Y_IN_M)/(MAX_Y_IN_M-MIN_Y_IN_M),5)
    writeLine("vt "+ str(xt) + " " + str(yt))    

writeLine("usemtl None")
writeLine("s 1")



for i in range(NUM_VERTICES-1):
    for j in range(NUM_VERTICES-1):
        indexOfSquareFaces = [verticesXY[i][j][0],verticesXY[i+1][j][0],verticesXY[i+1][j+1][0],verticesXY[i][j+1][0]]
        faces.append(indexOfSquareFaces)


    
for face in faces:
    tmp = [str(idx)+"/"+str(idx)+"/1" for idx in face]
    writeLine("f "+" ".join(tmp))

# print("===============")
# for face in faces:
#     for x in face:
#         print(vertices[x],"\t",end = '')
#     print()




f.close()


