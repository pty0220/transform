import vtk
import numpy as np


l2n = lambda l: np.array(l)
n2l = lambda n: list(n)


def read_skull(filename,  opacity):
    ren = vtk.vtkRenderer()




    readerstl = vtk.vtkSTLReader()
    readerstl.SetFileName(filename)
    readerstl.Update()

    reader = readerstl.GetOutput()


    STLmapper = vtk.vtkPolyDataMapper()
    STLmapper.SetInputData(reader)

    STLactor = vtk.vtkActor()
    STLactor.SetMapper(STLmapper)
    STLactor.GetProperty().SetOpacity(opacity)




    return reader, STLactor


def addLine(ren, p1, p2, color=[0.0, 0.0, 1.0], opacity=1.0):
    line = vtk.vtkLineSource()
    line.SetPoint1(p1)
    line.SetPoint2(p2)


    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(line.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(color)
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty()

    return line, actor


def addPoint(ren, p, color=[0.0,0.0,0.0], radius=0.5):

    point = vtk.vtkSphereSource()
    point.SetCenter(p)
    point.SetRadius(radius)
    point.SetPhiResolution(100)
    point.SetThetaResolution(100)


    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(point.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(color)



    return point, actor


def make_centerline_target(skull, target, centerline_length):
    pointLocator = vtk.vtkPointLocator()
    pointLocator.SetDataSet(skull)
    pointLocator.BuildLocator()
    id = pointLocator.FindClosestPoint(target)
    point_s = skull.GetPoint(id)

    vector = l2n(point_s) - l2n(target)
    centerline_vector = vector / np.linalg.norm(vector)
    centerline_target = n2l(l2n(target) + centerline_length * centerline_vector )

    return centerline_target, centerline_vector , point_s


def make_analysis_rage2(num_pts, radius, range_angle, opacity=0.25,centerline_vector=[0,0,0], Target=[0,0,0]):
    X= Target[0]
    Y =Target[1]
    Z= Target[2]

    # specify analysis range as  angle

    h_wid = radius*np.sin(np.deg2rad(range_angle))
    p_height = radius ** 2 - h_wid ** 2
    height_from_center = np.sqrt(p_height)
    height = radius - height_from_center  ### transducer's height
    rate = height / (radius * 2)  ## ratio height/radius*2

    indices_theta = np.arange(0, num_pts, dtype=float)
    indices_phi = np.linspace(0, num_pts * rate, num=num_pts)  ## define transdcuer's height as ratio

    phi = np.arccos(1 - 2 * indices_phi / num_pts)
    theta = np.pi * (1 + 5 ** 0.5) * indices_theta

    phi_deg = np.rad2deg(phi)

    x, y, z = np.cos(theta) * np.sin(phi)*radius, np.sin(theta) * np.sin(phi)*radius, np.cos(phi)*radius;

    # x,y,z is coordination of evenly distributed shpere
    # I will try to make poly data use this x,y,z*radius

    points = vtk.vtkPoints()


    for i in range(len(x)):
        array_point = np.array([x[i], y[i], z[i]] )
        points.InsertNextPoint(x[i],y[i],z[i])

    poly = vtk.vtkPolyData()
    poly.SetPoints(points)

    # To create surface of a sphere we need to use Delaunay triangulation
    d3D = vtk.vtkDelaunay3D()
    d3D.SetInputData( poly ) # This generates a 3D mesh

    # We need to extract the surface from the 3D mesh
    dss = vtk.vtkDataSetSurfaceFilter()
    dss.SetInputConnection( d3D.GetOutputPort() )
    dss.Update()

    # Now we have our final polydata
    spherePoly = dss.GetOutput()

    #rotation of analysis range

    center_vector = [1, 0, 0]
    unit_vector = centerline_vector / np.linalg.norm(centerline_vector)
    xy_unit_vector = l2n((unit_vector[0], unit_vector[1], 0))

    if n2l(xy_unit_vector) == [0, 0, 0]:
        xy_angle = 0.0
        z_angle = 90.0
    else:
        xy_angle = np.rad2deg(np.arccos(
            np.dot(center_vector, xy_unit_vector) / (np.linalg.norm(center_vector) * np.linalg.norm(xy_unit_vector))))
        z_angle = np.rad2deg(np.arccos(
            np.dot(xy_unit_vector, unit_vector) / (np.linalg.norm(xy_unit_vector) * np.linalg.norm(unit_vector))))
    if unit_vector[2] < 0:
        z_angle = -z_angle
    if unit_vector[1] < 0:
        xy_angle = -xy_angle

    #### transform (rotation)
    ##### translate first !!!! rotate second !!!!!!!!!!!!!!! important!!!!!

    transform = vtk.vtkTransform()
    transform.Translate(Target)

    transform.RotateWXYZ(90, 0, 1, 0)
    transform.RotateWXYZ(-xy_angle, 1, 0, 0)
    transform.RotateWXYZ(-z_angle, 0, 1, 0)

    transformFilter = vtk.vtkTransformPolyDataFilter()
    transformFilter.SetTransform(transform)
    transformFilter.SetInputData(spherePoly)
    transformFilter.Update()

    Cutpoly = transformFilter.GetOutput()


    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(transformFilter.GetOutput())

    mapper2 = vtk.vtkPolyDataMapper()
    mapper2.SetInputData(spherePoly)

    # cutting_center = n2l(l2n(Target) + l2n(centerline_vector)*(l2n(radius)*(np.sqrt(2)/2)))
    #
    # Plane = vtk.vtkPlane()
    # Plane.SetOrigin(cutting_center)
    # Plane.SetNormal(centerline_vector)
    #
    # Clipper = vtk.vtkClipPolyData()
    # Clipper.SetInputData(spherePoly)
    # Clipper.SetClipFunction(Plane)
    # Clipper.SetValue(0);
    # Clipper.Update()
    #
    # Cutpoly = Clipper.GetOutput()



    #
    # mapper = vtk.vtkPolyDataMapper()
    # mapper.SetInputData(Cutpoly)

    actor = vtk.vtkActor()
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetColor([0,0,1])
    actor.GetProperty().EdgeVisibilityOn()
    actor.GetProperty().SetEdgeColor([0,0,0])
    actor.SetMapper(mapper)

    return Cutpoly, actor


def make_analysis_rage(num_pts, radius, opacity=0.25,centerline_vector=[0,0,0], Target=[0,0,0]):
    X= Target[0]
    Y =Target[1]
    Z= Target[2]

    indices = np.arange(0, num_pts, dtype=float) + 0.5
    radius
    phi = np.arccos(1 - 2*indices/num_pts)
    theta = np.pi * (1 + 5**0.5) * indices

    x, y, z = np.cos(theta) * np.sin(phi)*radius+X, np.sin(theta) * np.sin(phi)*radius+Y, np.cos(phi)*radius+Z;

    # x,y,z is coordination of evenly distributed shpere
    # I will try to make poly data use this x,y,z*radius

    points = vtk.vtkPoints()


    for i in range(len(x)):
        array_point = np.array([x[i], y[i], z[i]] )
        points.InsertNextPoint(x[i],y[i],z[i])

    poly = vtk.vtkPolyData()
    poly.SetPoints(points)

    # To create surface of a sphere we need to use Delaunay triangulation
    d3D = vtk.vtkDelaunay3D()
    d3D.SetInputData( poly ) # This generates a 3D mesh

    # We need to extract the surface from the 3D mesh
    dss = vtk.vtkDataSetSurfaceFilter()
    dss.SetInputConnection( d3D.GetOutputPort() )
    dss.Update()

    # Now we have our final polydata
    spherePoly = dss.GetOutput()

    #calculate set point of analysis area
    cutting_center = n2l(l2n(Target) + l2n(centerline_vector)*(l2n(radius)*(np.sqrt(2)/2)))

    Plane = vtk.vtkPlane()
    Plane.SetOrigin(cutting_center)
    Plane.SetNormal(centerline_vector)

    Clipper = vtk.vtkClipPolyData()
    Clipper.SetInputData(spherePoly)
    Clipper.SetClipFunction(Plane)
    Clipper.SetValue(0);
    Clipper.Update()

    Cutpoly = Clipper.GetOutput()




    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(Cutpoly)

    actor = vtk.vtkActor()
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetColor([0,0,1])
    actor.GetProperty().EdgeVisibilityOn()
    actor.GetProperty().SetEdgeColor([0,0,0])
    actor.SetMapper(mapper)

    return Cutpoly, actor


def make_evencirle(num_pts = 1000, ROC=71,width=65,focal_length=55.22,range_vector=[0,0,0],Target=[0,0,0],opacity=0.7,color=[1,0,0]):
    ##################### make transducer function with evely distributed spots
    X = Target[0]
    Y = Target[1]
    Z = Target[2]

    h_wid = width / 2
    p_height = ROC ** 2 - h_wid ** 2
    height_from_center = np.sqrt(p_height)
    height = ROC - height_from_center  ### transducer's height
    rate = height / (ROC * 2)  ## ratio height/ROC*2

    indices_theta = np.arange(0, num_pts, dtype=float)
    indices_phi = np.linspace(0, num_pts * rate, num=num_pts)   ## define transdcuer's height as ratio

    phi = np.arccos(1 - 2 * indices_phi / num_pts)
    theta = np.pi * (1 + 5 ** 0.5) * indices_theta

    phi_deg = np.rad2deg(phi)

    x, y, z = np.cos(theta) * np.sin(phi) * ROC + X, np.sin(theta) * np.sin(phi) * ROC + Y, np.cos(
        phi) * ROC + Z;

    # x,y,z is coordination of evenly distributed shpere
    # I will try to make poly data use this x,y,z*radius

    points = vtk.vtkPoints()


    for i in range(len(x)):

        points.InsertNextPoint(x[i],y[i],z[i])

    poly = vtk.vtkPolyData()
    poly.SetPoints(points)

    # To create surface of a sphere we need to use Delaunay triangulation
    d3D = vtk.vtkDelaunay3D()
    d3D.SetInputData( poly ) # This generates a 3D mesh

    # We need to extract the surface from the 3D mesh
    dss = vtk.vtkDataSetSurfaceFilter()
    dss.SetInputConnection( d3D.GetOutputPort() )
    dss.Update()

    # Now we have our final polydata
    spherePoly = dss.GetOutput()


    return spherePoly


def make_transducer(spherePoly, ROC=71, width=65, focal_length=55.22, range_vector=[0, 0, 0], Target=[0, 0, 0],
                    opacity=1, color=[1, 0, 0]):


    center_vector = [1, 0, 0]
    unit_vector = range_vector / np.linalg.norm(range_vector)
    xy_unit_vector = l2n((unit_vector[0], unit_vector[1], 0))

    if n2l(xy_unit_vector) == [0, 0, 0]:
        xy_angle = 0.0
        z_angle = 90.0
    else:
        xy_angle = np.rad2deg(np.arccos(
            np.dot(center_vector, xy_unit_vector) / (np.linalg.norm(center_vector) * np.linalg.norm(xy_unit_vector))))
        z_angle = np.rad2deg(np.arccos(
            np.dot(xy_unit_vector, unit_vector) / (np.linalg.norm(xy_unit_vector) * np.linalg.norm(unit_vector))))
    if unit_vector[2] < 0:
        z_angle = -z_angle
    if unit_vector[1] < 0:
        xy_angle = -xy_angle

    #### transform (rotation)

    gap = focal_length - ROC
    GAP = n2l(l2n(Target) + l2n(range_vector) * gap)



    transform = vtk.vtkTransform()
    transform.Translate(GAP)    #### move to the gap(trandcuer center to target) point
    transform.RotateWXYZ(90, 0, 1, 0)
    transform.RotateWXYZ(-xy_angle, 1, 0, 0)
    transform.RotateWXYZ(-z_angle, 0, 1, 0)

    transformFilter = vtk.vtkTransformPolyDataFilter()
    transformFilter.SetTransform(transform)
    transformFilter.SetInputData(spherePoly)
    transformFilter.Update()

    Transducer = transformFilter.GetOutput()


    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(transformFilter.GetOutput())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().EdgeVisibilityOn()
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetColor(color)


    # gap = focal_length - ROC
    #
    # GAP = n2l(l2n(Target) + l2n(range_vector) * gap)
    #
    # # calculate cutting height
    # h_wid = width / 2
    #
    # p_height = ROC ** 2 - h_wid ** 2
    # height = np.sqrt(p_height)
    #
    # # calculate set point of analysis area
    # cutting_center = n2l(l2n(range_vector) * height)
    #
    # Plane = vtk.vtkPlane()
    # Plane.SetOrigin(cutting_center)
    # Plane.SetNormal(range_vector)
    #
    # Clipper = vtk.vtkClipPolyData()
    # Clipper.SetInputData(spherePoly)
    # Clipper.SetClipFunction(Plane)
    # Clipper.SetValue(0);
    # Clipper.Update()
    #
    # Cutpoly = Clipper.GetOutput()
    #
    # transform = vtk.vtkTransform()
    # transform.Translate(GAP)
    # transformFilter = vtk.vtkTransformPolyDataFilter()
    # transformFilter.SetTransform(transform)
    # transformFilter.SetInputData(Cutpoly)
    # transformFilter.Update()
    #
    # Transducer = transformFilter.GetOutput()
    #
    # mapper = vtk.vtkPolyDataMapper()
    # mapper.SetInputData(Transducer)
    #
    # actor = vtk.vtkActor()

    # actor.GetProperty().EdgeVisibilityOn()
    # actor.GetProperty().SetEdgeColor([0, 0, 0])
    # actor.SetMapper(mapper)

    return Transducer, actor, xy_angle, z_angle

def cut_skull(skull,Target, centerline_vector,opacity=0.3):
    cutting_center = n2l(l2n(Target) - l2n(centerline_vector)*15)

    Plane = vtk.vtkPlane()
    Plane.SetOrigin(cutting_center)
    Plane.SetNormal(centerline_vector)



    source = vtk.vtkCylinder()
    source.SetCenter(Target)
    source.SetRadius(50.0)

    Clipper = vtk.vtkClipPolyData()
    Clipper.SetInputData(skull)
    Clipper.SetClipFunction(Plane)
    Clipper.SetValue(0);
    Clipper.Update()

    skull_cut = Clipper.GetOutput()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(skull_cut)

    actor = vtk.vtkActor()
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetColor([1, 0, 0])
    actor.GetProperty().EdgeVisibilityOff()
    actor.GetProperty().SetEdgeColor([0, 0, 0])
    actor.SetMapper(mapper)

    return skull_cut, actor


def cut_skull_loop(skull,Target, centerline_vector,opacity=0.7):
    cutting_center = n2l(l2n(Target) + l2n(centerline_vector)*5)

    Plane = vtk.vtkPlane()
    Plane.SetOrigin(Target)
    Plane.SetNormal(centerline_vector)



    source = vtk.vtkCylinder()
    source.SetCenter(Target)
    source.SetRadius(50.0)

    Clipper = vtk.vtkClipPolyData()
    Clipper.SetInputData(skull)
    Clipper.SetClipFunction(Plane)
    Clipper.SetValue(0);
    Clipper.Update()

    skull_cut = Clipper.GetOutput()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(skull_cut)

    actor = vtk.vtkActor()
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetColor([0.4, 0.4, 0.4])
    actor.GetProperty().EdgeVisibilityOff()
    actor.GetProperty().SetEdgeColor([0, 0, 0])
    actor.SetMapper(mapper)

    return skull_cut, actor


def translate (ren,move_point ,vtkobject,opacity, color ):

    transform = vtk.vtkTransform()
    #transform.RotateWXYZ(90, 0, 1, 0)
    transform.Translate(move_point)    #### move to the gap(trandcuer center to target) point

    #transform.RotateWXYZ(67.24592159420198811, 0, 1, 0)
    #transform.RotateWXYZ(-6.23172765980718246, 1, 0, 0)

    #transform.Translate(move_point)    #### move to the gap(trandcuer center to target) point


    transformFilter = vtk.vtkTransformPolyDataFilter()
    transformFilter.SetTransform(transform)

    if vtkobject.GetClassName() == 'vtkPolyData':
        transformFilter.SetInputData(vtkobject)
    else:
        transformFilter.SetInputConnection(vtkobject.GetOutputPort())

    transformFilter.Update()

    move_poly = transformFilter.GetOutput()



    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(transformFilter.GetOutput())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().EdgeVisibilityOn()
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetColor(color)
    actor.GetProperty().EdgeVisibilityOff()

    return move_poly, actor


def rotate (ren,vector,angle ,vtkobject,opacity, color ):

    transform = vtk.vtkTransform()
    #transform.RotateWXYZ(90, 0, 1, 0)
   #transform.Translate(move_point)    #### move to the gap(trandcuer center to target) point
    transform.RotateWXYZ(angle, vector[0], vector[1], vector[2])


    #transform.RotateWXYZ(67.24592159420198811, 0, 1, 0)
    #transform.RotateWXYZ(90, 0, 0, 1)
    #transform.Translate(move_point)    #### move to the gap(trandcuer center to target) point


    transformFilter = vtk.vtkTransformPolyDataFilter()
    transformFilter.SetTransform(transform)

    if vtkobject.GetClassName() == 'vtkPolyData':
        transformFilter.SetInputData(vtkobject)
    else:
        transformFilter.SetInputConnection(vtkobject.GetOutputPort())

    transformFilter.Update()

    move_poly = transformFilter.GetOutput()


    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(transformFilter.GetOutput())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().EdgeVisibilityOn()
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetColor(color)
    actor.GetProperty().EdgeVisibilityOff()

    return move_poly, actor


