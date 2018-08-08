import vtk
import helpfunction as pty
import numpy as np

l2n = lambda l: np.array(l)
n2l = lambda n: list(n)

ren = vtk.vtkRenderer()
axes = vtk.vtkAxesActor()
axes.SetTotalLength(30,30,30)

#######################################################################################
######################################target point
D_tran2target =55.22

M1 =[-35.684, -40.853, 46.613]
SMA= [22.7292, -27.1715, 56.572]
DLPFC= [52.973, 20.717, 29.100]
FEF =[-36.455, 13.735, 41.150]

right_FEF_K = [61.960, -0.8, 28.140]
left_FEF_k = [-41.96, 4.28, 34.25]

right_FEF_Y = [61.20, 8.10, 46.48]
left_FEF_Y = [-28.08, 4.08, 62.31]

DMPFC =[4.968, 27.067,28.192]
Cingulate  = [13.695, 25.115, 25.136]

Target = SMA

#######################################################################################
########################################


readtxt = np.loadtxt("SMA ARC result & step 0")
starting_point = readtxt[:,4:7]
skull, skull_actor = pty.read_skull('skull-smooth2.stl',0.5)
space, space_actor = pty.read_skull('brain_space.stl',1)
focus, focus_actor = pty.addPoint(ren,Target,[1,0,0],4)



cylinder = vtk.vtkCylinderSource()
cylinder.SetResolution(4)
cylinder.SetRadius(25)
cylinder.SetHeight(25)

cylinder.SetCenter(0,0,0)

R_cylinder, R_cylinder_actor = pty.rotate(ren, (1,0,0),90,cylinder,1,[0,0,1])
RR_cylinder, RR_cylinder_actor = pty.rotate(ren, (0,0,1),45,R_cylinder,1,[0,0,1])
T_cylinder, T_cylinder_actor = pty.translate(ren,(0,0,55.22),RR_cylinder,1,[0,0,1])



sphere_Tri = vtk.vtkTriangleFilter()
sphere_Tri.SetInputData(T_cylinder)


booleanOperation = vtk.vtkBooleanOperationPolyDataFilter()
booleanOperation.SetOperationToIntersection()

booleanOperation.SetInputConnection(1, sphere_Tri.GetOutputPort())
booleanOperation.SetInputData(0, skull)
booleanOperation.Update()

booleanOperationMapper = vtk.vtkPolyDataMapper()
booleanOperationMapper.SetInputConnection(booleanOperation.GetOutputPort())
booleanOperationMapper.ScalarVisibilityOff()


booleanOperationActor = vtk.vtkActor()
booleanOperationActor.SetMapper(booleanOperationMapper)



connectivityFilter = vtk.vtkPolyDataConnectivityFilter()
connectivityFilter.SetInputConnection(booleanOperation.GetOutputPort())
connectivityFilter.SetExtractionModeToSpecifiedRegions()
connectivityFilter.AddSpecifiedRegion(1)
connectivityFilter.Update()

extractedMapper = vtk.vtkPolyDataMapper()
extractedMapper.SetInputConnection(connectivityFilter.GetOutputPort())
extractedMapper.ScalarVisibilityOff()

extractedActor = vtk.vtkActor()
extractedActor.SetMapper(extractedMapper)
extractedActor.GetProperty().SetColor(0,1,1)








#cylinderActor.RotateX(30.0)
#cylinderActor.RotateY(-45.0)






for i in range(1):


    start, start_actor = pty.addPoint(ren,starting_point[i,:],[0,0,1],4)
    line, line_actor = pty.addLine(ren,starting_point[i,:],Target,[0,0,1],1)


    Target_n = l2n(Target)
    negative_Target = n2l(-Target_n)
    a = readtxt[i,1]


    T_skull, T_skull_actor = pty.translate(ren,negative_Target, skull, 0.5, [0.8,0.8,0.8])
    T_focus, T_focus_actor = pty.translate(ren, negative_Target, focus, 1, [1,0,0])
    T_start, T_start_actor = pty.translate(ren,negative_Target,start, 1, [0,0,1] )
    T_line, T_line_actor = pty.translate(ren,negative_Target,line, 1, [0,0,1] )


    R_skull, R_skull_actor = pty.rotate(ren,[0,0,1],-readtxt[i,2] ,T_skull, 0.5, [0.8,0.8,0.8])
    R_focus, R_focus_actor = pty.rotate(ren,[0,0,1],-readtxt[i,2], T_focus, 1, [1,0,0])
    R_start, R_start_actor = pty.rotate(ren,[0,0,1],-readtxt[i,2], T_start, 1, [0,0,1])
    R_line , R_line_actor  = pty.rotate(ren,[0,0,1],-readtxt[i,2], T_line , 1, [0,0,1])


    R_skull2, R_skull_actor2 = pty.rotate(ren, [0,1,0],readtxt[i,3], R_skull, 0.5, [0.8,0.8,0.8])
    R_focus2, R_focus_actor2 = pty.rotate(ren, [0,1,0],readtxt[i,3], R_focus, 1, [1,0,0])
    R_start2, R_start_actor2 = pty.rotate(ren, [0,1,0],readtxt[i,3], R_start, 1, [0,0,1] )
    R_line2, R_line_actor2   = pty.rotate(ren, [0,1,0],readtxt[i,3], R_line , 1, [0,0,1] )


    R_skull3, R_skull_actor3 = pty.rotate(ren, [0,1,0], 90, R_skull2, 0.5, [0.8,0.8,0.8])
    R_focus3, R_focus_actor3 = pty.rotate(ren, [0,1,0], 90, R_focus2, 1, [1,0,0] )
    R_start3, R_start_actor3 = pty.rotate(ren, [0,1,0], 90, R_start2, 1, [0,0,1] )
    R_line3, R_line_actor3   = pty.rotate(ren, [0,1,0], 90, R_line2 , 1, [0,0,1] )


    T_skull2, T_skull_actor2 = pty.translate(ren, [0, 0, D_tran2target], R_skull3, 0.5, [0.8,0.8,0.8])
    T_focus2, T_focus_actor2 = pty.translate(ren, [0, 0, D_tran2target], R_focus3, 1, [1,0,0])
    T_start2, T_start_actor2 = pty.translate(ren, [0, 0, D_tran2target],R_start3, 1, [0,0,1] )
    T_line2, T_line_actor2   = pty.translate(ren, [0, 0, D_tran2target],  R_line3, 1, [0,0,1] )

    #writer = vtk.vtkPolyDataWriter()
    #writer.SetInputData(T_skull2)
    #writer.SetFileName('SMA_result_transform' +str(i) + '.vtk')
    #writer.Update()




    # ren.AddActor(axes)
    #
    #
    #ren.AddActor(T_skull_actor2)
    #ren.AddActor(T_focus_actor2)
    # ren.AddActor(T_start_actor2)
    # ren.AddActor(T_line_actor2)


#ren.AddActor(T_skull_actor)
# ren.AddActor(R_focus_actor)
# ren.AddActor(R_start_actor)
# ren.AddActor(R_line_actor)

# ren.AddActor(R_skull_actor2)
# ren.AddActor(R_focus_actor2)
# ren.AddActor(R_start_actor2)
# ren.AddActor(R_line_actor2)

#
# ren.AddActor(R_skull_actor3)
# ren.AddActor(R_focus_actor3)
# ren.AddActor(R_start_actor3)
# ren.AddActor(R_line_actor3)

#ren.AddActor(T_cylinder_actor)
#ren.AddActor(axes)
#ren.AddActor(booleanOperationActor)
#ren.AddActor(extractedActor)


ren.AddActor(T_cylinder_actor)
ren.AddActor(skull_actor)
ren.AddActor(focus_actor)
# ren.AddActor(start_actor)
ren.AddActor(line_actor)




ren.SetBackground(0.2, 0.2, 0.2)
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)

iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)

iren.Initialize()
renWin.Render()
iren.Start()
