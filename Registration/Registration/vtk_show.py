import vtk

def load_stl_file(file_path):
    """
    加载 STL 文件并返回一个 vtkPolyData 对象
    """
    reader = vtk.vtkSTLReader()
    reader.SetFileName(file_path)
    reader.Update()
    return reader.GetOutput()

def create_renderer():
    """
    创建一个渲染器并设置背景颜色
    """
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.1, 0.2, 0.3)  # 设置背景颜色为深蓝色
    return renderer

def create_render_window(renderer, window_title="STL Viewer"):
    """
    创建一个渲染窗口
    """
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    render_window.SetWindowName(window_title)
    return render_window

def create_interactor(render_window):
    """
    创建一个交互器
    """
    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(render_window)
    return interactor

def add_actor(renderer, poly_data,color = 0):
    """
    将 STL 数据添加到渲染器中
    """
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly_data)
    
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    if color == 1:
        actor.GetProperty().SetColor((1.0, 1.0, 1.0))  # 设置模型颜色
    renderer.AddActor(actor)


    
def vtk_show(path):
    stl_file_path1 = path
    poly_data1 = load_stl_file(stl_file_path1)
    renderer = create_renderer()
    render_window = create_render_window(renderer)
    interactor = create_interactor(render_window)
	
    # 将两个 STL 数据添加到渲染器中
    # 可以为每个模型设置不同的颜色
    add_actor(renderer, poly_data1, color=(1.0, 0.0, 0.0))  # 外侧部分为红色
    interactor.Initialize()
    interactor.Start()

def main():
    # 加载 STL 文件
    stl_file_path1 = "合并后的STL.stl"
    stl_file_path2 = "合并后的STL.stl"
    poly_data1 = load_stl_file(stl_file_path1)
    poly_data2 = load_stl_file(stl_file_path2)

    # 创建渲染器、渲染窗口和交互器
    renderer = create_renderer()
    render_window = create_render_window(renderer)
    interactor = create_interactor(render_window)
	
    # 将两个 STL 数据添加到渲染器中
    # 可以为每个模型设置不同的颜色
    add_actor(renderer, poly_data1, color=(1.0, 0.0, 0.0))  # 外侧部分为红色
    add_actor(renderer, poly_data2, color=(0.0, 1.0, 0.0))  # 内侧部分为绿色

    # 启动交互器
    interactor.Initialize()
    interactor.Start()

if __name__ == "__main__":
    main()