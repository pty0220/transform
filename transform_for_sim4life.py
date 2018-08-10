import vtk
import helpfunction as pty
import numpy as np
from brain_target import*

import tkinter.filedialog as tk
import os



l2n = lambda l: np.array(l)
n2l = lambda n: list(n)

ren = vtk.vtkRenderer()
axes = vtk.vtkAxesActor()
axes.SetTotalLength(30,30,30)

#######################################################################################
######################################target point
D_tran2target =55.22


Target = S1_re
Target_name = 'S1'

#######################################################################################
########################################
print("Choose ARC calculation result")
txt_file_name = tk.askopenfilename()
print("Choose save directory")
txt_folder_name = tk.askdirectory()

readtxt = np.loadtxt(txt_file_name)
ARC = readtxt[:,0]

starting_point = readtxt[:,4:7]
skull, skull_actor = pty.read_skull('skull-smooth2.stl',0.5)
space, space_actor = pty.read_skull('brain_space.stl',1)
focus, focus_actor = pty.addPoint(ren,Target,[1,0,0],4)


angle   =  input('type range angle: ')
number   = input('type whole transducer number: ')

skull_name = Target_name + '_' + number + '_' + angle + '_skull_transform'
space_name = Target_name + '_' + number + '_' + angle + '_space_transform'

os.chdir(txt_folder_name)
for i in range(1200):

    number = readtxt[i, 1]
    print(number)
    if ARC[i] < 1:


        start, start_actor = pty.addPoint(ren,starting_point[i,:],[0,0,1],4)
        line, line_actor = pty.addLine(ren,starting_point[i,:],Target,[0,0,1],1)


        Target_n = l2n(Target)
        negative_Target = n2l(-Target_n)
        a = readtxt[i,1]


        T_skull, T_skull_actor = pty.translate(ren,negative_Target, skull, 0.5, [0.8,0.8,0.8])
        T_focus, T_focus_actor = pty.translate(ren, negative_Target, focus, 1, [1,0,0])
        T_start, T_start_actor = pty.translate(ren,negative_Target,start, 1, [0,0,1] )
        T_line, T_line_actor = pty.translate(ren,negative_Target,line, 1, [0,0,1] )
        T_space, T_space_actor = pty.translate(ren, negative_Target,space,1,[0,1,1])


        R_skull, R_skull_actor = pty.rotate(ren,[0,0,1],-readtxt[i,2] ,T_skull, 0.5, [0.8,0.8,0.8])
        R_focus, R_focus_actor = pty.rotate(ren,[0,0,1],-readtxt[i,2], T_focus, 1, [1,0,0])
        R_start, R_start_actor = pty.rotate(ren,[0,0,1],-readtxt[i,2], T_start, 1, [0,0,1])
        R_line , R_line_actor  = pty.rotate(ren,[0,0,1],-readtxt[i,2], T_line , 1, [0,0,1])
        R_space, R_space_actor = pty.rotate(ren,[0,0,1],-readtxt[i,2], T_space, 1, [0,1,1])

        R_skull2, R_skull_actor2 = pty.rotate(ren, [0,1,0],readtxt[i,3], R_skull, 0.5, [0.8,0.8,0.8])
        R_focus2, R_focus_actor2 = pty.rotate(ren, [0,1,0],readtxt[i,3], R_focus, 1, [1,0,0])
        R_start2, R_start_actor2 = pty.rotate(ren, [0,1,0],readtxt[i,3], R_start, 1, [0,0,1] )
        R_line2, R_line_actor2   = pty.rotate(ren, [0,1,0],readtxt[i,3], R_line , 1, [0,0,1] )
        R_space2, R_space_actor2 = pty.rotate(ren, [0,1,0],readtxt[i,3], R_space, 1, [0,1,1])


        R_skull3, R_skull_actor3 = pty.rotate(ren, [0,1,0], 90, R_skull2, 0.5, [0.8,0.8,0.8])
        R_focus3, R_focus_actor3 = pty.rotate(ren, [0,1,0], 90, R_focus2, 1, [1,0,0] )
        R_start3, R_start_actor3 = pty.rotate(ren, [0,1,0], 90, R_start2, 1, [0,0,1] )
        R_line3, R_line_actor3   = pty.rotate(ren, [0,1,0], 90, R_line2 , 1, [0,0,1] )
        R_space3, R_space_acotr3 = pty.rotate(ren, [0,1,0], 90, R_space2, 1, [0,1,1])


        T_skull2, T_skull_actor2 = pty.translate(ren, [0, 0, D_tran2target], R_skull3, 0.5, [0.8,0.8,0.8])
        T_focus2, T_focus_actor2 = pty.translate(ren, [0, 0, D_tran2target], R_focus3, 1, [1,0,0])
        T_start2, T_start_actor2 = pty.translate(ren, [0, 0, D_tran2target], R_start3, 1, [0,0,1] )
        T_line2, T_line_actor2   = pty.translate(ren, [0, 0, D_tran2target], R_line3,  1, [0,0,1] )
        T_space2, T_space_actor2 = pty.translate(ren, [0, 0, D_tran2target], R_space3, 1,[0,1,1])




        writer = vtk.vtkPolyDataWriter()
        writer.SetInputData(T_skull2)
        name_num = skull_name+str(i)
        writer.SetFileName(name_num)
        writer.Update()

        writer2 = vtk.vtkPolyDataWriter()
        s_name_num = space_name+str(i)
        writer.SetFileName(s_name_num)
        writer.Update()

        print(str(i)+ 'number finish')





# ren.AddActor(axes)
#     #
# ren.AddActor(T_space_actor2)
# ren.AddActor(T_skull_actor2)
# ren.AddActor(T_focus_actor2)
#
# ren.AddActor(T_line_actor2)


#ren.AddActor(T_skull_actor)
#ren.AddActor(T_space_actor)
# ren.AddActor(R_focus_actor)
# ren.AddActor(R_start_actor)
# ren.AddActor(R_line_actor)

#ren.AddActor(T_skull_actor2)
#ren.AddActor(T_space_actor2)
# ren.AddActor(R_focus_actor2)
# ren.AddActor(R_start_actor2)
# ren.AddActor(R_line_actor2)

#
# ren.AddActor(R_skull_actor3)
# ren.AddActor(R_focus_actor3)
# ren.AddActor(R_start_actor3)
# ren.AddActor(R_line_actor3)


#ren.AddActor(skull_actor)
#ren.AddActor(space_actor)
# ren.AddActor(focus_actor)
# ren.AddActor(start_actor)
# ren.AddActor(line_actor)


#

# ren.SetBackground(0.2, 0.2, 0.2)
# renWin = vtk.vtkRenderWindow()
# renWin.AddRenderer(ren)
#
# iren = vtk.vtkRenderWindowInteractor()
# iren.SetRenderWindow(renWin)
#
# iren.Initialize()
# renWin.Render()
# iren.Start()
