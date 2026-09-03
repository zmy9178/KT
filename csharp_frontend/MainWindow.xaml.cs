using System;
using System.Collections.Generic;
using System.Data;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using IoPath = System.IO.Path;

namespace AirdropFrontend;

public partial class MainWindow : Window
{
    private readonly double[] _defaultKte = [0, 800, 320, 0, 5.75, -1.24, 6, 7, 25];
    private double[] _vel = [15.2, 4.96e4, 2.64e4, -3.279, -0.407, 5.6, 3.2, 2.2, 0.506, 4, 1.614, -0.34, -0.698, -1.209, -4.9, -5.75, -4.9, 0.83, 0.05];
    private double[] _qys = [20.2, 0.8, 14, 5.55];
    private double[] _jss = [10, 0.8, 6.8, 0.01, 2, 8, 22, 4, 1, 8, 0.9];
    private double[] _zhs = [760, 9, 58.767, 0.026, 6.618, 1, 0.8, 0.8, 0.8, 0.6, 200, 56, 24.734, 80.425, 1, 0.16, 23.5, 4.3, 5];

    private DataTable? _firstResult;
    private DataTable? _secondResult;
    private bool _isAirdropPage;

    private TextBox? _dataDirBox;
    private TextBox? _pythonBox;
    private TextBox? _scriptBox;
    private TextBox? _simTimeBox;
    private TextBox? _simStepBox;
    private TextBox? _airfieldAltitudeBox;
    private TextBox? _dropHeightBox;
    private TextBox? _aircraftSpeedBox;
    private TextBox? _cabinAngleBox;
    private TextBox? _loadXBox;
    private TextBox? _loadYBox;
    private TextBox? _windXBox;
    private TextBox? _windYBox;
    private TextBox? _groundTempBox;
    private TextBlock? _statusBlock;
    private ListBox? _curveList;
    private Canvas? _plotCanvas;
    private double _plotZoomX = 1.0;
    private double _plotZoomY = 1.0;
    private double _plotPanX;
    private double _plotPanY;
    private bool _isPlotPanning;
    private Point _plotPanStart;
    private double _plotStartPanX;
    private double _plotStartPanY;
    private DataGrid? _resultGrid;
    private Button? _runFirstButton;
    private Button? _runSecondButton;
    private Button? _runAllButton;
    private TextBox? _endSpeedBox;
    private TextBox? _endXSpeedBox;
    private TextBox? _endYSpeedBox;
    private TextBox? _endZSpeedBox;
    private TextBox? _endPitchBox;
    private TextBox? _endRollBox;
    private TextBox? _endYawBox;
    private TextBox? _maxLoadBox;

    private string DataDir => _dataDirBox?.Text.Trim() ?? IoPath.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "KTData");
    private string PythonCommand => string.IsNullOrWhiteSpace(_pythonBox?.Text) ? "python" : _pythonBox!.Text.Trim();
    private string ScriptPath => string.IsNullOrWhiteSpace(_scriptBox?.Text) ? FindPythonBridgeScript() : _scriptBox!.Text.Trim();

    public MainWindow()
    {
        InitializeComponent();
    }

    private void Window_Loaded(object sender, RoutedEventArgs e)
    {
        ShowLogin();
    }

    private void ShowLogin()
    {
        Root.Children.Clear();
        var page = new Grid { Background = TryBackground("login_background.jpg", SkyBrush()) };
        var scene = new Canvas { IsHitTestVisible = false };
        scene.Children.Add(ShapePath("M 1000,88 C 1095,28 1190,88 1190,170 C 1280,160 1360,220 1360,300 C 1215,285 1080,285 940,300 C 905,215 925,135 1000,88 Z", "#88FFFFFF"));
        scene.Children.Add(ShapePath("M 145,250 L 520,150 L 610,210 L 330,300 L 480,390 L 420,420 L 235,325 L 90,370 L 58,336 Z", "#22558BC0"));
        scene.Children.Add(new Line { X1 = 1125, Y1 = 250, X2 = 1118, Y2 = 565, Stroke = Brushes.White, StrokeThickness = 2, Opacity = 0.55 });
        scene.Children.Add(new Ellipse { Width = 26, Height = 36, Fill = new SolidColorBrush(Color.FromRgb(43, 66, 82)), Margin = new Thickness(1105, 560, 0, 0) });
        page.Children.Add(scene);

        var panel = new Border
        {
            Width = 830,
            Height = 360,
            Background = new SolidColorBrush(Color.FromArgb(88, 255, 255, 255)),
            BorderBrush = new SolidColorBrush(Color.FromArgb(150, 255, 255, 255)),
            BorderThickness = new Thickness(1),
            HorizontalAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center
        };
        var grid = new Grid { Margin = new Thickness(42) };
        AddRows(grid, GridLength.Auto, new GridLength(28), GridLength.Auto, GridLength.Auto, GridLength.Auto);
        var title = Text("空降特种车辆空投过程仿真与评估软件V2026.1", 30, FontWeights.Bold, Brushes.White);
        title.HorizontalAlignment = HorizontalAlignment.Center;
        title.Effect = new System.Windows.Media.Effects.DropShadowEffect { Color = Color.FromRgb(18, 64, 96), BlurRadius = 5, ShadowDepth = 2 };
        grid.Children.Add(title);
        grid.Children.Add(LoginRow("用户名", new TextBox { Height = 40, FontSize = 16 }, 2));
        grid.Children.Add(LoginRow("密码", new PasswordBox { Height = 40, FontSize = 16 }, 3));
        var buttons = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Center, Margin = new Thickness(0, 34, 0, 0) };
        Grid.SetRow(buttons, 4);
        buttons.Children.Add(Button("注册", NotImplemented_Click, 118, 42));
        buttons.Children.Add(Button("登录", (_, _) => ShowHome(), 118, 42));
        buttons.Children.Add(Button("用户管理", NotImplemented_Click, 118, 42));
        buttons.Children.Add(Button("软件配置", (_, _) => ShowConfigDialog(), 118, 42));
        grid.Children.Add(buttons);
        panel.Child = grid;
        page.Children.Add(panel);
        Root.Children.Add(page);
    }

    private UIElement LoginRow(string label, Control editor, int row)
    {
        var grid = new Grid { Width = 560, HorizontalAlignment = HorizontalAlignment.Center, Margin = new Thickness(0, row == 2 ? 8 : 20, 0, 0) };
        AddCols(grid, new GridLength(100), new GridLength(1, GridUnitType.Star));
        grid.Children.Add(Text(label, 16, FontWeights.Normal, Brushes.White, HorizontalAlignment.Right, new Thickness(0, 0, 18, 0)));
        Grid.SetColumn(editor, 1);
        grid.Children.Add(editor);
        Grid.SetRow(grid, row);
        return grid;
    }

    private void ShowHome()
    {
        Root.Children.Clear();
        var page = new Grid { Background = TryBackground("home_background.jpg", HomeBrush()) };
        var scene = new Canvas { IsHitTestVisible = false };
        scene.Children.Add(ShapePath("M 200,280 L 520,165 L 612,225 L 350,350 L 458,470 L 398,500 L 252,376 L 118,422 L 82,388 Z", "#AA0B4D88"));
        scene.Children.Add(new Ellipse { Width = 260, Height = 120, Fill = new SolidColorBrush(Color.FromArgb(220, 255, 255, 255)), Margin = new Thickness(905, 195, 0, 0) });
        scene.Children.Add(new Ellipse { Width = 230, Height = 112, Fill = new SolidColorBrush(Color.FromArgb(210, 255, 255, 255)), Margin = new Thickness(1030, 230, 0, 0) });
        scene.Children.Add(new Line { X1 = 1030, Y1 = 310, X2 = 1010, Y2 = 670, Stroke = Brushes.White, StrokeThickness = 2, Opacity = 0.55 });
        scene.Children.Add(new Line { X1 = 1135, Y1 = 330, X2 = 1010, Y2 = 670, Stroke = Brushes.White, StrokeThickness = 2, Opacity = 0.55 });
        page.Children.Add(scene);

        var dock = new DockPanel { Margin = new Thickness(42) };
        var title = Text("空降特种车辆空投过程仿真与评估V2026.1", 34, FontWeights.Bold, Brushes.White);
        title.HorizontalAlignment = HorizontalAlignment.Center;
        title.Margin = new Thickness(0, 28, 0, 42);
        title.Effect = new System.Windows.Media.Effects.DropShadowEffect { Color = Color.FromRgb(18, 64, 96), BlurRadius = 5, ShadowDepth = 2 };
        DockPanel.SetDock(title, Dock.Top);
        dock.Children.Add(title);

        var menu = new UniformGrid { Columns = 3, Rows = 3, HorizontalAlignment = HorizontalAlignment.Center, VerticalAlignment = VerticalAlignment.Center };
        menu.Children.Add(HomeButton("出舱\n过程仿真", (_, _) => OpenSimulation(false)));
        menu.Children.Add(HomeButton("空降\n过程仿真", (_, _) => OpenSimulation(true)));
        menu.Children.Add(HomeButton("着陆缓冲\n过程仿真", (_, _) => OpenAnalysis("着陆缓冲过程仿真")));
        menu.Children.Add(HomeButton("着陆缓冲\n等效建模", (_, _) => OpenAnalysis("着陆缓冲等效建模")));
        menu.Children.Add(HomeButton("模型准确度\n标定与修正", (_, _) => OpenAnalysis("模型准确度标定与修正")));
        menu.Children.Add(HomeButton("空投过程\n可视化", (_, _) => OpenVisualization_Click(this, new RoutedEventArgs())));
        menu.Children.Add(HomeButton("空投\n适应性评估", (_, _) => OpenAnalysis("空投适应性评估")));
        menu.Children.Add(HomeButton("软件配置", (_, _) => ShowConfigDialog()));
        menu.Children.Add(HomeButton("退出登录", (_, _) => ShowLogin()));
        dock.Children.Add(menu);
        page.Children.Add(dock);
        Root.Children.Add(page);
    }

    private void OpenSimulation(bool airdrop)
    {
        _isAirdropPage = airdrop;
        Root.Children.Clear();
        var dock = new DockPanel { Background = new SolidColorBrush(Color.FromRgb(241, 242, 244)) };
        dock.Children.Add(BuildHeader(airdrop ? "空降过程仿真" : "出舱过程仿真"));

        var main = new Grid { Margin = new Thickness(10) };
        AddCols(main, new GridLength(320), new GridLength(1, GridUnitType.Star), new GridLength(420));
        main.Children.Add(BuildLeftPane(airdrop));
        var center = BuildCenterPane(airdrop);
        Grid.SetColumn(center, 1);
        main.Children.Add(center);
        var right = BuildRightPane();
        Grid.SetColumn(right, 2);
        main.Children.Add(right);
        dock.Children.Add(main);
        Root.Children.Add(dock);
        RefreshOutputs();
    }

    private UIElement BuildHeader(string title)
    {
        var border = new Border { Background = Brushes.White, BorderBrush = BorderBrush(), BorderThickness = new Thickness(0, 0, 0, 1), Padding = new Thickness(10, 8, 10, 8) };
        DockPanel.SetDock(border, Dock.Top);
        var grid = new Grid();
        AddCols(grid, GridLength.Auto, new GridLength(1, GridUnitType.Star), GridLength.Auto);
        grid.Children.Add(Text($"空降特种车辆空投过程仿真与评估V2026.1-{title}", 15, FontWeights.SemiBold, Brushes.Black));
        _statusBlock = Text("就绪", 13, FontWeights.Normal, new SolidColorBrush(Color.FromRgb(70, 84, 106)));
        _statusBlock.Margin = new Thickness(18, 0, 0, 0);
        Grid.SetColumn(_statusBlock, 1);
        grid.Children.Add(_statusBlock);
        var actions = new StackPanel { Orientation = Orientation.Horizontal };
        actions.Children.Add(Button("软件配置", (_, _) => ShowConfigDialog()));
        actions.Children.Add(Button("返回主界面", (_, _) => ShowHome()));
        actions.Children.Add(Button("下一步", (_, _) => OpenSimulation(!_isAirdropPage)));
        Grid.SetColumn(actions, 2);
        grid.Children.Add(actions);
        border.Child = grid;
        return border;
    }

    private UIElement BuildLeftPane(bool airdrop)
    {
        var grid = new Grid();
        AddRows(grid, new GridLength(2, GridUnitType.Star), new GridLength(2, GridUnitType.Star), GridLength.Auto, GridLength.Auto);
        var first = ModelGroup(airdrop ? "主伞参数设置" : "空降车仿真模型选择", airdrop ? ["1. 伞降模型", "2. 主伞改进模型"] : ["A车", "B车"], airdrop ? EditMainChute : EditVehicle);
        grid.Children.Add(first);
        var second = ModelGroup(airdrop ? "减速伞参数设置" : "牵引伞仿真模型选择", airdrop ? ["XX型号伞", "XX改进型"] : ["XX型号伞", "XX改进型"], airdrop ? EditDecelerationChute : EditPilotChute);
        Grid.SetRow(second, 1);
        grid.Children.Add(second);
        var dir = Section("当前目录", ConfigGrid(), null);
        Grid.SetRow(dir, 2);
        grid.Children.Add(dir);
        var info = Section("模型信息", ModelInfoGrid(airdrop), null);
        Grid.SetRow(info, 3);
        grid.Children.Add(info);
        return grid;
    }

    private UIElement ModelGroup(string title, string[] items, Action editAction)
    {
        var dock = new DockPanel();
        var list = new ListBox { Height = 185 };
        foreach (var item in items)
        {
            list.Items.Add(new ListBoxItem { Content = item, IsSelected = list.Items.Count == 0 });
        }
        DockPanel.SetDock(list, Dock.Top);
        dock.Children.Add(list);
        var buttons = new UniformGrid { Columns = 4, Margin = new Thickness(0, 8, 0, 0) };
        buttons.Children.Add(Button("新建", (_, _) => editAction()));
        buttons.Children.Add(Button("编辑", (_, _) => editAction()));
        buttons.Children.Add(Button("删除", NotImplemented_Click));
        buttons.Children.Add(Button("选择", (_, _) => SetStatus("已选择模型")));
        DockPanel.SetDock(buttons, Dock.Bottom);
        dock.Children.Add(buttons);
        return Section(title, dock, null);
    }

    private UIElement ConfigGrid()
    {
        var grid = new Grid();
        AddCols(grid, new GridLength(72), new GridLength(1, GridUnitType.Star));
        AddRows(grid, GridLength.Auto, GridLength.Auto, GridLength.Auto);
        _dataDirBox = Box(IoPath.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "KTData"));
        _pythonBox = Box("python");
        _scriptBox = Box(FindPythonBridgeScript());
        AddLabelBox(grid, "数据目录", _dataDirBox, 0);
        AddLabelBox(grid, "Python", _pythonBox, 1);
        AddLabelBox(grid, "脚本", _scriptBox, 2);
        return grid;
    }

    private UIElement ModelInfoGrid(bool airdrop)
    {
        var grid = new Grid();
        AddCols(grid, new GridLength(72), new GridLength(1, GridUnitType.Star));
        AddRows(grid, GridLength.Auto, GridLength.Auto, GridLength.Auto);
        AddLabelBox(grid, "模型用途", Box(airdrop ? "空降过程仿真" : "出舱过程仿真"), 0);
        AddLabelBox(grid, "保存位置", Box("本地参数"), 1);
        AddLabelBox(grid, "创建时间", Box("2026-09-02"), 2);
        return grid;
    }

    private UIElement BuildCenterPane(bool airdrop)
    {
        var grid = new Grid();
        AddRows(grid, new GridLength(150), new GridLength(1, GridUnitType.Star), new GridLength(176), GridLength.Auto);
        grid.Children.Add(Section(airdrop ? "初始参数设置" : "参数设置", InitialGrid(), null));
        var curves = Section("曲线选择", CurveList(airdrop), null);
        Grid.SetRow(curves, 1);
        grid.Children.Add(curves);
        var env = Section(airdrop ? "仿真工况设置 / 空投环境参数设置" : "仿真工况设置", EnvironmentGrid(), null);
        Grid.SetRow(env, 2);
        grid.Children.Add(env);
        var actions = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Center, Margin = new Thickness(0, 8, 0, 0) };
        _runFirstButton = Button("出舱仿真", RunFirst_Click);
        _runSecondButton = Button("空降仿真", RunSecond_Click);
        _runAllButton = Button("完整流程", RunAll_Click);
        actions.Children.Add(_runFirstButton);
        actions.Children.Add(_runSecondButton);
        actions.Children.Add(_runAllButton);
        actions.Children.Add(Button("提取结果", Refresh_Click));
        actions.Children.Add(Button("过程可视化", OpenVisualization_Click));
        Grid.SetRow(actions, 3);
        grid.Children.Add(actions);
        return grid;
    }

    private UIElement InitialGrid()
    {
        var grid = new Grid();
        AddCols(grid, new GridLength(90), new GridLength(1, GridUnitType.Star), new GridLength(50), new GridLength(105), new GridLength(1, GridUnitType.Star), new GridLength(50));
        AddRows(grid, GridLength.Auto, GridLength.Auto, GridLength.Auto, GridLength.Auto);
        AddInput(grid, "纵向速度", Box("0"), "m/s", 0, 0);
        AddInput(grid, "横向速度", Box("0"), "m/s", 1, 0);
        AddInput(grid, "垂向速度", Box("0"), "m/s", 2, 0);
        AddInput(grid, "侧倾角", Box("0"), "deg", 3, 0);
        AddInput(grid, "俯仰角", Box("0"), "deg", 0, 3);
        AddInput(grid, "纵向坡度", Box("0"), "deg", 1, 3);
        AddInput(grid, "侧倾坡度", Box("0"), "deg", 2, 3);
        _simTimeBox = Box("50");
        AddInput(grid, "仿真时间", _simTimeBox, "s", 3, 3);
        return grid;
    }

    private UIElement CurveList(bool airdrop)
    {
        _curveList = new ListBox();
        var labels = airdrop
            ? new[] { "空降车x向位移", "空降车y向位移", "空降车z向位移", "空降车x向速度", "空降车y向速度", "空降车z向速度", "空降车俯仰角", "空降车侧倾角", "开伞过程变化曲线" }
            : new[] { "空降车x向位移", "空降车y向位移", "空降车x向速度", "空降车y向速度", "空降车俯仰角", "空降车俯仰角速度" };
        foreach (var label in labels)
        {
            _curveList.Items.Add(new ListBoxItem { Content = label, IsSelected = _curveList.Items.Count == 0 });
        }
        _curveList.SelectionChanged += (_, _) => ResetPlotView();
        return _curveList;
    }

    private UIElement EnvironmentGrid()
    {
        var grid = new Grid();
        AddCols(grid, new GridLength(116), new GridLength(1, GridUnitType.Star), new GridLength(42), new GridLength(116), new GridLength(1, GridUnitType.Star), new GridLength(42));
        AddRows(grid, GridLength.Auto, GridLength.Auto, GridLength.Auto, GridLength.Auto, GridLength.Auto);
        _simStepBox = Box("0.001");
        _airfieldAltitudeBox = Box(_defaultKte[0].ToString(CultureInfo.InvariantCulture));
        _dropHeightBox = Box(_defaultKte[1].ToString(CultureInfo.InvariantCulture));
        _aircraftSpeedBox = Box(_defaultKte[2].ToString(CultureInfo.InvariantCulture));
        _cabinAngleBox = Box(_defaultKte[3].ToString(CultureInfo.InvariantCulture));
        _loadXBox = Box(_defaultKte[4].ToString(CultureInfo.InvariantCulture));
        _loadYBox = Box(_defaultKte[5].ToString(CultureInfo.InvariantCulture));
        _windXBox = Box(_defaultKte[6].ToString(CultureInfo.InvariantCulture));
        _windYBox = Box(_defaultKte[7].ToString(CultureInfo.InvariantCulture));
        _groundTempBox = Box(_defaultKte[8].ToString(CultureInfo.InvariantCulture));
        AddInput(grid, "仿真步长", _simStepBox, "s", 0, 0);
        AddInput(grid, "着陆场海拔", _airfieldAltitudeBox, "m", 0, 3);
        AddInput(grid, "空投相对高度", _dropHeightBox, "m", 1, 0);
        AddInput(grid, "运输机速度", _aircraftSpeedBox, "km/h", 1, 3);
        AddInput(grid, "货舱角度", _cabinAngleBox, "deg", 2, 0);
        AddInput(grid, "装载位置X", _loadXBox, "m", 2, 3);
        AddInput(grid, "装载位置Y", _loadYBox, "m", 3, 0);
        AddInput(grid, "地面水平风速", _windXBox, "m/s", 3, 3);
        AddInput(grid, "地面侧向风速", _windYBox, "m/s", 4, 0);
        AddInput(grid, "着陆场气温", _groundTempBox, "C", 4, 3);
        return grid;
    }

    private UIElement BuildRightPane()
    {
        var grid = new Grid();
        AddRows(grid, new GridLength(155), new GridLength(2, GridUnitType.Star), new GridLength(2, GridUnitType.Star));
        grid.Children.Add(Section("仿真结果数值显示", ResultSummaryGrid(), null));
        var plotPanel = new DockPanel();
        var plotToolbar = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Right, Margin = new Thickness(0, 0, 0, 6) };
        plotToolbar.Children.Add(Button("重置视图", (_, _) => ResetPlotView(), 86, 26));
        DockPanel.SetDock(plotToolbar, Dock.Top);
        plotPanel.Children.Add(plotToolbar);
        _plotCanvas = new Canvas
        {
            Background = new SolidColorBrush(Color.FromRgb(250, 251, 252)),
            ClipToBounds = true,
            Cursor = Cursors.Cross
        };
        _plotCanvas.SizeChanged += (_, _) => { ClampPlotView(); DrawPlot(); };
        _plotCanvas.MouseWheel += PlotCanvas_MouseWheel;
        _plotCanvas.MouseLeftButtonDown += PlotCanvas_MouseLeftButtonDown;
        _plotCanvas.MouseLeftButtonUp += PlotCanvas_MouseLeftButtonUp;
        _plotCanvas.MouseMove += PlotCanvas_MouseMove;
        _plotCanvas.MouseLeave += (_, _) => EndPlotPan();
        plotPanel.Children.Add(_plotCanvas);
        var plot = Section("仿真曲线", plotPanel, null);
        Grid.SetRow(plot, 1);
        grid.Children.Add(plot);
        _resultGrid = new DataGrid { AutoGenerateColumns = true, IsReadOnly = true, CanUserAddRows = false };
        var table = Section("仿真数据表格", _resultGrid, null);
        Grid.SetRow(table, 2);
        grid.Children.Add(table);
        return grid;
    }

    private UIElement ResultSummaryGrid()
    {
        var grid = new Grid();
        AddCols(grid, new GridLength(92), new GridLength(1, GridUnitType.Star), new GridLength(40), new GridLength(92), new GridLength(1, GridUnitType.Star), new GridLength(40));
        AddRows(grid, GridLength.Auto, GridLength.Auto, GridLength.Auto, GridLength.Auto);
        _endSpeedBox = Box("", true);
        _endXSpeedBox = Box("", true);
        _endYSpeedBox = Box("", true);
        _endZSpeedBox = Box("", true);
        _endPitchBox = Box("", true);
        _endRollBox = Box("", true);
        _endYawBox = Box("", true);
        _maxLoadBox = Box("0", true);
        AddInput(grid, "着陆末速度", _endSpeedBox, "m/s", 0, 0);
        AddInput(grid, "最大过载", _maxLoadBox, "g", 0, 3);
        AddInput(grid, "纵向末速度", _endXSpeedBox, "m/s", 1, 0);
        AddInput(grid, "末俯仰角", _endPitchBox, "deg", 1, 3);
        AddInput(grid, "横向末速度", _endYSpeedBox, "m/s", 2, 0);
        AddInput(grid, "末侧倾角", _endRollBox, "deg", 2, 3);
        AddInput(grid, "垂向末速度", _endZSpeedBox, "m/s", 3, 0);
        AddInput(grid, "末偏航角", _endYawBox, "deg", 3, 3);
        return grid;
    }

    private void OpenAnalysis(string title)
    {
        Root.Children.Clear();
        var dock = new DockPanel { Background = new SolidColorBrush(Color.FromRgb(241, 242, 244)) };
        dock.Children.Add(AnalysisHeader(title));
        var main = new Grid { Margin = new Thickness(10) };
        AddCols(main, new GridLength(310), new GridLength(1, GridUnitType.Star), new GridLength(430));
        main.Children.Add(AnalysisLeft(title));
        var center = AnalysisCenter(title);
        Grid.SetColumn(center, 1);
        main.Children.Add(center);
        var right = AnalysisRight(title);
        Grid.SetColumn(right, 2);
        main.Children.Add(right);
        dock.Children.Add(main);
        Root.Children.Add(dock);
    }

    private UIElement AnalysisHeader(string title)
    {
        var border = new Border { Background = Brushes.White, BorderBrush = BorderBrush(), BorderThickness = new Thickness(0, 0, 0, 1), Padding = new Thickness(10, 8, 10, 8) };
        DockPanel.SetDock(border, Dock.Top);
        var grid = new Grid();
        AddCols(grid, new GridLength(1, GridUnitType.Star), GridLength.Auto);
        grid.Children.Add(Text($"空降特种车辆空投过程仿真与评估V2026.1-{title}", 15, FontWeights.SemiBold, Brushes.Black));
        var actions = new StackPanel { Orientation = Orientation.Horizontal };
        actions.Children.Add(Button("返回主界面", (_, _) => ShowHome()));
        actions.Children.Add(Button("下一步", (_, _) => OpenSimulation(false)));
        Grid.SetColumn(actions, 1);
        grid.Children.Add(actions);
        border.Child = grid;
        return border;
    }

    private UIElement AnalysisLeft(string title)
    {
        var stack = new StackPanel();
        stack.Children.Add(AnalysisList("模型列表", ["1. 伞降模型", "2. 着陆缓冲等效模型", "3. 着陆缓冲中等效模型"]));
        stack.Children.Add(AnalysisList(title.Contains("适应") ? "历史数据" : "试验数据列表", ["1. xx", "2. xx", "3. xx"]));
        var info = new Grid();
        AddCols(info, new GridLength(78), new GridLength(1, GridUnitType.Star));
        AddRows(info, GridLength.Auto, GridLength.Auto, GridLength.Auto);
        AddLabelBox(info, "模型用途", Box(title), 0);
        AddLabelBox(info, "保存位置", Box("本地项目"), 1);
        AddLabelBox(info, "创建时间", Box("2026-09-02"), 2);
        stack.Children.Add(Section("模型信息", info, 130));
        return stack;
    }

    private UIElement AnalysisList(string title, string[] rows)
    {
        var dock = new DockPanel();
        var list = new ListBox();
        foreach (var row in rows) list.Items.Add(row);
        dock.Children.Add(list);
        var actions = new UniformGrid { Columns = 4 };
        actions.Children.Add(Button("导入", NotImplemented_Click));
        actions.Children.Add(Button("编辑", NotImplemented_Click));
        actions.Children.Add(Button("删除", NotImplemented_Click));
        actions.Children.Add(Button("选择", NotImplemented_Click));
        DockPanel.SetDock(actions, Dock.Bottom);
        dock.Children.Add(actions);
        return Section(title, dock, 230);
    }

    private UIElement AnalysisCenter(string title)
    {
        var grid = new Grid();
        AddRows(grid, new GridLength(150), new GridLength(1, GridUnitType.Star), new GridLength(150), GridLength.Auto);
        grid.Children.Add(Section(title.Contains("适应") ? "定义变量" : "初始参数设置", AnalysisInputGrid(), null));
        var table = new DataGrid { AutoGenerateColumns = false };
        table.Columns.Add(new DataGridTextColumn { Header = "序号" });
        table.Columns.Add(new DataGridTextColumn { Header = "节点编号" });
        table.Columns.Add(new DataGridTextColumn { Header = "测点位置" });
        var measure = Section(title.Contains("适应") ? "历史数据 / 确定型统计计算" : "后处理设置 / 测点设置", table, null);
        Grid.SetRow(measure, 1);
        grid.Children.Add(measure);
        var lower = Section(title.Contains("适应") ? "空投成功率评估" : "模型参数修正", AnalysisCorrectionGrid(title), null);
        Grid.SetRow(lower, 2);
        grid.Children.Add(lower);
        var buttons = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Center };
        foreach (var text in title.Contains("适应") ? new[] { "开始计算", "中止计算", "保存数据", "开始评估", "显示结果" } : new[] { "仿真结果加载", "试验结果加载", "参数灵敏度分析", "优化计算", "模型准确度计算" })
        {
            buttons.Children.Add(Button(text, NotImplemented_Click));
        }
        Grid.SetRow(buttons, 3);
        grid.Children.Add(buttons);
        return grid;
    }

    private UIElement AnalysisInputGrid()
    {
        var grid = new Grid();
        AddCols(grid, new GridLength(100), new GridLength(1, GridUnitType.Star), new GridLength(60), new GridLength(110), new GridLength(1, GridUnitType.Star), new GridLength(60));
        AddRows(grid, GridLength.Auto, GridLength.Auto, GridLength.Auto);
        AddInput(grid, "纵向速度", Box("0"), "m/s", 0, 0);
        AddInput(grid, "横向速度", Box("0"), "m/s", 1, 0);
        AddInput(grid, "垂向速度", Box("0"), "m/s", 2, 0);
        AddInput(grid, "俯仰角", Box("0"), "deg", 0, 3);
        AddInput(grid, "侧倾坡度", Box("0"), "deg", 1, 3);
        AddInput(grid, "仿真时间", Box("50"), "s", 2, 3);
        return grid;
    }

    private UIElement AnalysisCorrectionGrid(string title)
    {
        var grid = new Grid();
        AddCols(grid, new GridLength(140), new GridLength(1, GridUnitType.Star), new GridLength(140), new GridLength(1, GridUnitType.Star));
        AddRows(grid, GridLength.Auto, GridLength.Auto, GridLength.Auto, GridLength.Auto);
        if (title.Contains("适应"))
        {
            AddLabelBox(grid, "评估量阈值", Box("710"), 0);
            AddLabelBox(grid, "成功率阈值", Box("99.5"), 1);
            AddLabelBox(grid, "安全投放总次数", Box("20"), 2);
            AddLabelBox(grid, "本次成功率", Box("100"), 3);
        }
        else
        {
            AddLabelBox(grid, "优化目标编号", Box(""), 0);
            AddLabelBox(grid, "优化目标值", Box(""), 1);
            AddLabelBox(grid, "优化后值", Box(""), 2);
            AddLabelBox(grid, "模型准确度", Box(""), 3);
        }
        return grid;
    }

    private UIElement AnalysisRight(string title)
    {
        var grid = new Grid();
        AddRows(grid, new GridLength(150), new GridLength(1, GridUnitType.Star), new GridLength(1, GridUnitType.Star));
        var top = new UniformGrid { Columns = 3 };
        top.Children.Add(Text("曲线名称", 12)); top.Children.Add(Box("")); top.Children.Add(Text("最大值 0 MPa", 12));
        top.Children.Add(Text("节点位置", 12)); top.Children.Add(Box("")); top.Children.Add(Text("对应时间 0 ms", 12));
        grid.Children.Add(Section(title.Contains("适应") ? "评估结果" : "仿真结果数值显示", top, null));
        var p1 = Section(title.Contains("适应") ? "仿真曲线" : "仿真结果曲线显示", CenterText("示意图区"), null);
        Grid.SetRow(p1, 1);
        grid.Children.Add(p1);
        var p2 = Section(title.Contains("适应") ? "试验曲线 / 对比曲线" : "计算过程信息", CenterText("曲线显示区域"), null);
        Grid.SetRow(p2, 2);
        grid.Children.Add(p2);
        return grid;
    }

    private async void RunFirst_Click(object sender, RoutedEventArgs e) => await RunStageAsync("first");
    private async void RunSecond_Click(object sender, RoutedEventArgs e) => await RunStageAsync("second");
    private async void RunAll_Click(object sender, RoutedEventArgs e) => await RunStageAsync("all");
    private void Refresh_Click(object sender, RoutedEventArgs e) => RefreshOutputs();
    private void OpenVisualization_Click(object sender, RoutedEventArgs e) => OpenAnalysis("空投过程可视化");
    private void NotImplemented_Click(object sender, RoutedEventArgs e) => MessageBox.Show("该功能为前端预留入口，后续接入数据管理逻辑。", "功能预留", MessageBoxButton.OK, MessageBoxImage.Information);

    private async Task RunStageAsync(string stage)
    {
        SetBusy(true, $"正在运行 {stage}...");
        try
        {
            WriteInputFiles();
            string output = await Task.Run(() => RunPython(stage));
            RefreshOutputs();
            SetStatus(string.IsNullOrWhiteSpace(output) ? "运行完成" : output.Trim());
        }
        catch (Exception ex)
        {
            SetStatus("错误：" + ex.Message);
            MessageBox.Show(ex.Message, "运行失败", MessageBoxButton.OK, MessageBoxImage.Error);
        }
        finally
        {
            SetBusy(false);
        }
    }

    private void WriteInputFiles()
    {
        Directory.CreateDirectory(DataDir);
        var sim = new[] { ParseBox(_simTimeBox, "仿真时间"), ParseBox(_simStepBox, "仿真步长") };
        var kte = new[]
        {
            ParseBox(_airfieldAltitudeBox, "着陆场海拔"), ParseBox(_dropHeightBox, "空投相对高度"), ParseBox(_aircraftSpeedBox, "运输机速度"),
            ParseBox(_cabinAngleBox, "货舱角度"), ParseBox(_loadXBox, "装载位置X"), ParseBox(_loadYBox, "装载位置Y"),
            ParseBox(_windXBox, "地面水平风速"), ParseBox(_windYBox, "地面侧向风速"), ParseBox(_groundTempBox, "着陆场气温")
        };
        WriteTextAtomically(IoPath.Combine(DataDir, "airdrop_input_first.csv"), string.Join(Environment.NewLine, Row("vel_p", _vel), Row("qys_p", _qys), Row("sim_p", sim)));
        WriteTextAtomically(IoPath.Combine(DataDir, "airdrop_input_second.csv"), string.Join(Environment.NewLine, Row("kte_p", kte), Row("jss_p", _jss), Row("zhs_p", _zhs)));
    }

    private static string Row(string name, IEnumerable<double> values) => name + "," + string.Join(",", values.Select(v => v.ToString("G17", CultureInfo.InvariantCulture)));

    private string RunPython(string stage)
    {
        if (!File.Exists(ScriptPath)) throw new FileNotFoundException("找不到 Python 桥接脚本。", ScriptPath);
        var psi = new ProcessStartInfo
        {
            FileName = PythonCommand,
            Arguments = $"\"{ScriptPath}\" {stage} --data-dir \"{DataDir}\"",
            WorkingDirectory = IoPath.GetDirectoryName(ScriptPath) ?? Environment.CurrentDirectory,
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            CreateNoWindow = true,
            StandardOutputEncoding = Encoding.UTF8,
            StandardErrorEncoding = Encoding.UTF8,
        };
        using var process = Process.Start(psi) ?? throw new InvalidOperationException("Python 进程启动失败。");
        string stdout = process.StandardOutput.ReadToEnd();
        string stderr = process.StandardError.ReadToEnd();
        process.WaitForExit();
        if (process.ExitCode != 0) throw new InvalidOperationException(string.IsNullOrWhiteSpace(stderr) ? stdout : stderr);
        return stdout;
    }

    private void RefreshOutputs()
    {
        _firstResult = LoadCsvIfExists(IoPath.Combine(DataDir, "airdrop_output_first.csv"));
        _secondResult = LoadCsvIfExists(IoPath.Combine(DataDir, "airdrop_output_second.csv"));
        var table = _isAirdropPage ? _secondResult : _firstResult;
        if (_resultGrid is not null) _resultGrid.ItemsSource = table?.DefaultView;
        UpdateSummary(table);
        ResetPlotView(false);
        DrawPlot();
        SetStatus("结果已刷新");
    }

    private void UpdateSummary(DataTable? table)
    {
        if (table is null || table.Rows.Count == 0) return;
        var last = table.Rows[table.Rows.Count - 1];
        double vx = Get(last, table, "x_velocity");
        double vy = Get(last, table, "y_velocity");
        double vz = Get(last, table, "z_velocity");
        Set(_endXSpeedBox, vx); Set(_endYSpeedBox, vy); Set(_endZSpeedBox, vz);
        Set(_endSpeedBox, Math.Sqrt(vx * vx + vy * vy + vz * vz));
        Set(_endPitchBox, Get(last, table, "pitch_angle"));
        Set(_endRollBox, Get(last, table, "roll_angle"));
        Set(_endYawBox, Get(last, table, "yaw_angle"));
        if (_maxLoadBox is not null && table.Columns.Contains("rope_force"))
        {
            double maxLoad = table.Rows.Cast<DataRow>()
                .Where(row => row["rope_force"] is not DBNull)
                .Select(row => Math.Abs(Convert.ToDouble(row["rope_force"])))
                .DefaultIfEmpty(0.0)
                .Max();
            Set(_maxLoadBox, maxLoad);
        }
    }

    private static double Get(DataRow row, DataTable table, string name) => table.Columns.Contains(name) && row[name] is not DBNull ? Convert.ToDouble(row[name]) : 0.0;
    private static void Set(TextBox? box, double value) { if (box is not null) box.Text = value.ToString("G6", CultureInfo.InvariantCulture); }

    private static DataTable? LoadCsvIfExists(string path)
    {
        if (!File.Exists(path)) return null;
        var lines = File.ReadAllLines(path, Encoding.UTF8);
        if (lines.Length == 0) return null;
        var headers = lines[0].Split(',').Select(x => x.Trim().Trim('"')).ToArray();
        var table = new DataTable(IoPath.GetFileNameWithoutExtension(path));
        foreach (var header in headers) table.Columns.Add(header, typeof(double));
        foreach (string line in lines.Skip(1))
        {
            if (string.IsNullOrWhiteSpace(line)) continue;
            var cells = line.Split(',');
            var row = table.NewRow();
            for (int i = 0; i < headers.Length && i < cells.Length; i++)
                row[i] = double.TryParse(cells[i], NumberStyles.Float, CultureInfo.InvariantCulture, out double v) ? v : DBNull.Value;
            table.Rows.Add(row);
        }
        return table;
    }

    private static void WriteTextAtomically(string path, string content)
    {
        string tempPath = IoPath.Combine(IoPath.GetDirectoryName(path) ?? Environment.CurrentDirectory, IoPath.GetFileNameWithoutExtension(path) + ".tmp" + IoPath.GetExtension(path));
        File.WriteAllText(tempPath, content + Environment.NewLine, new UTF8Encoding(true));
        File.Move(tempPath, path, overwrite: true);
    }

    private void DrawPlot()
    {
        if (_plotCanvas is null) return;
        _plotCanvas.Children.Clear();
        var table = _isAirdropPage ? _secondResult : _firstResult;
        string? column = SelectedColumnName();
        if (table is null || column is null || !table.Columns.Contains(column) || table.Rows.Count < 2)
        {
            AddPlotLabel("暂无可绘制数据", 18, 18);
            return;
        }
        var points = new List<Point>();
        bool hasTime = table.Columns.Contains("time");
        for (int i = 0; i < table.Rows.Count; i++)
        {
            var row = table.Rows[i];
            if (row[column] is DBNull) continue;
            if (hasTime && row["time"] is DBNull) continue;
            double x = hasTime ? Convert.ToDouble(row["time"]) : i;
            double y = Convert.ToDouble(row[column]);
            points.Add(new Point(x, y));
        }
        if (points.Count < 2) { AddPlotLabel("暂无可绘制数据", 18, 18); return; }
        if (!TryGetPlotArea(out double left, out double top, out double plotW, out double plotH)) return;
        ClampPlotView();
        double minX = points.Min(p => p.X), maxX = points.Max(p => p.X);
        double minY = points.Min(p => p.Y), maxY = points.Max(p => p.Y);
        if (Math.Abs(maxX - minX) < 1e-12) maxX = minX + 1;
        if (Math.Abs(maxY - minY) < 1e-12) maxY = minY + 1;

        _plotCanvas.Children.Add(new Line { X1 = left, Y1 = top, X2 = left, Y2 = top + plotH, Stroke = Brushes.LightSlateGray, StrokeThickness = 1 });
        _plotCanvas.Children.Add(new Line { X1 = left, Y1 = top + plotH, X2 = left + plotW, Y2 = top + plotH, Stroke = Brushes.LightSlateGray, StrokeThickness = 1 });
        var border = new Rectangle
        {
            Width = plotW,
            Height = plotH,
            Stroke = new SolidColorBrush(Color.FromRgb(219, 224, 232)),
            StrokeThickness = 1
        };
        Canvas.SetLeft(border, left);
        Canvas.SetTop(border, top);
        _plotCanvas.Children.Add(border);

        var layer = new Canvas { Clip = new RectangleGeometry(new Rect(left, top, plotW, plotH)) };
        var pts = new PointCollection();
        foreach (var point in points)
        {
            double xNorm = (point.X - minX) / (maxX - minX);
            double yNorm = 1 - (point.Y - minY) / (maxY - minY);
            pts.Add(new Point(left + _plotPanX + plotW * _plotZoomX * xNorm, top + _plotPanY + plotH * _plotZoomY * yNorm));
        }
        layer.Children.Add(new Polyline { Points = pts, Stroke = new SolidColorBrush(Color.FromRgb(27, 108, 147)), StrokeThickness = 2 });
        _plotCanvas.Children.Add(layer);
        AddPlotLabel($"{column}   t={minX:G6}-{maxX:G6}s   min={minY:G6}   max={maxY:G6}   zoom={_plotZoomX:G3}x/{_plotZoomY:G3}x", left, 2);
    }

    private void PlotCanvas_MouseWheel(object sender, MouseWheelEventArgs e)
    {
        if (_plotCanvas is null || !TryGetPlotArea(out double left, out double top, out double plotW, out double plotH)) return;
        var point = e.GetPosition(_plotCanvas);
        double factor = e.Delta > 0 ? 1.25 : 0.8;
        bool verticalOnly = (Keyboard.Modifiers & ModifierKeys.Control) == ModifierKeys.Control;
        bool horizontalOnly = (Keyboard.Modifiers & ModifierKeys.Shift) == ModifierKeys.Shift;
        bool zoomX = !verticalOnly;
        bool zoomY = !horizontalOnly;

        if (zoomX)
        {
            double anchorX = (point.X - left - _plotPanX) / (plotW * _plotZoomX);
            double nextZoomX = Math.Clamp(_plotZoomX * factor, 1.0, 200.0);
            _plotPanX = point.X - left - anchorX * plotW * nextZoomX;
            _plotZoomX = nextZoomX;
        }
        if (zoomY)
        {
            double anchorY = (point.Y - top - _plotPanY) / (plotH * _plotZoomY);
            double nextZoomY = Math.Clamp(_plotZoomY * factor, 1.0, 200.0);
            _plotPanY = point.Y - top - anchorY * plotH * nextZoomY;
            _plotZoomY = nextZoomY;
        }

        ClampPlotView();
        DrawPlot();
        e.Handled = true;
    }

    private void PlotCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        if (_plotCanvas is null) return;
        _isPlotPanning = true;
        _plotPanStart = e.GetPosition(_plotCanvas);
        _plotStartPanX = _plotPanX;
        _plotStartPanY = _plotPanY;
        _plotCanvas.Cursor = Cursors.SizeAll;
        _plotCanvas.CaptureMouse();
    }

    private void PlotCanvas_MouseMove(object sender, MouseEventArgs e)
    {
        if (!_isPlotPanning || _plotCanvas is null) return;
        var point = e.GetPosition(_plotCanvas);
        _plotPanX = _plotStartPanX + point.X - _plotPanStart.X;
        _plotPanY = _plotStartPanY + point.Y - _plotPanStart.Y;
        ClampPlotView();
        DrawPlot();
    }

    private void PlotCanvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e) => EndPlotPan();

    private void EndPlotPan()
    {
        if (_plotCanvas is null) return;
        _isPlotPanning = false;
        _plotCanvas.Cursor = Cursors.Cross;
        _plotCanvas.ReleaseMouseCapture();
    }

    private void ResetPlotView(bool redraw = true)
    {
        _plotZoomX = 1.0;
        _plotZoomY = 1.0;
        _plotPanX = 0;
        _plotPanY = 0;
        if (redraw) DrawPlot();
    }

    private void ClampPlotView()
    {
        if (!TryGetPlotArea(out _, out _, out double plotW, out double plotH)) return;
        _plotZoomX = Math.Clamp(_plotZoomX, 1.0, 200.0);
        _plotZoomY = Math.Clamp(_plotZoomY, 1.0, 200.0);
        _plotPanX = Math.Clamp(_plotPanX, plotW - plotW * _plotZoomX, 0);
        _plotPanY = Math.Clamp(_plotPanY, plotH - plotH * _plotZoomY, 0);
    }

    private bool TryGetPlotArea(out double left, out double top, out double plotW, out double plotH)
    {
        double width = Math.Max(_plotCanvas?.ActualWidth ?? 0, 260);
        double height = Math.Max(_plotCanvas?.ActualHeight ?? 0, 180);
        left = 48;
        top = 24;
        const double right = 20, bottom = 36;
        plotW = width - left - right;
        plotH = height - top - bottom;
        return plotW > 1 && plotH > 1;
    }

    private string? SelectedColumnName()
    {
        var label = (_curveList?.SelectedItem as ListBoxItem)?.Content?.ToString() ?? "";
        var map = new Dictionary<string, string>
        {
            ["空降车x向位移"] = "x_position", ["空降车y向位移"] = "y_position", ["空降车z向位移"] = "z_position",
            ["空降车x向速度"] = "x_velocity", ["空降车y向速度"] = "y_velocity", ["空降车z向速度"] = "z_velocity",
            ["空降车俯仰角"] = "pitch_angle", ["空降车侧倾角"] = "roll_angle", ["开伞过程变化曲线"] = "rope_force",
            ["空降车俯仰角速度"] = "pitch_angle"
        };
        return map.TryGetValue(label, out string? column) ? column : null;
    }

    private void AddPlotLabel(string text, double x, double y)
    {
        if (_plotCanvas is null) return;
        var label = Text(text, 12, FontWeights.Normal, new SolidColorBrush(Color.FromRgb(86, 96, 112)));
        Canvas.SetLeft(label, x); Canvas.SetTop(label, y);
        _plotCanvas.Children.Add(label);
    }

    private void EditVehicle() { var next = EditVector("空降车参数设置", VehicleDefs(), _vel); if (next is not null) _vel = next; }
    private void EditPilotChute() { var next = EditVector("牵引伞参数设置", PilotDefs(), _qys); if (next is not null) _qys = next; }
    private void EditDecelerationChute() { var next = EditVector("减速伞参数设置", DecelerationDefs(), _jss); if (next is not null) _jss = next; }
    private void EditMainChute() { var next = EditVector("主伞参数设置", MainChuteDefs(), _zhs, 19, 20); if (next is not null) _zhs = next; }

    private double[]? EditVector(string title, ParamDef[] defs, double[] values, params int[] allowedLengths)
    {
        allowedLengths = allowedLengths.Length == 0 ? [defs.Length] : allowedLengths;
        var source = values.Length >= defs.Length ? values.Take(defs.Length).ToArray() : values.Concat(Enumerable.Repeat(0.0, defs.Length - values.Length)).ToArray();
        double[]? result = null;
        var win = new Window { Title = "新建" + title, Owner = this, Width = 660, Height = Math.Min(760, 150 + defs.Length * 42), WindowStartupLocation = WindowStartupLocation.CenterOwner };
        var dock = new DockPanel { Margin = new Thickness(22) };
        var titleBlock = Text(title, 18, FontWeights.Bold, Brushes.Black);
        DockPanel.SetDock(titleBlock, Dock.Top);
        dock.Children.Add(titleBlock);
        var form = new Grid { Margin = new Thickness(20, 18, 20, 12) };
        AddCols(form, new GridLength(210), new GridLength(1, GridUnitType.Star), new GridLength(64));
        for (int i = 0; i < defs.Length; i++) AddRows(form, GridLength.Auto);
        var boxes = new List<TextBox>();
        for (int i = 0; i < defs.Length; i++)
        {
            var box = Box(source[i].ToString("G17", CultureInfo.InvariantCulture));
            boxes.Add(box);
            AddInput(form, defs[i].Name, box, defs[i].Unit, i, 0);
        }
        var scroll = new ScrollViewer { Content = form, VerticalScrollBarVisibility = ScrollBarVisibility.Auto };
        DockPanel.SetDock(scroll, Dock.Top);
        dock.Children.Add(scroll);
        var actions = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Center };
        actions.Children.Add(Button("确定", (_, _) =>
        {
            result = boxes.Select((b, i) => ParseBox(b, defs[i].Name)).ToArray();
            win.DialogResult = true;
        }, 110, 34));
        actions.Children.Add(Button("取消", (_, _) => win.Close(), 110, 34));
        DockPanel.SetDock(actions, Dock.Bottom);
        dock.Children.Add(actions);
        win.Content = dock;
        win.ShowDialog();
        return result;
    }

    private static ParamDef[] VehicleDefs() =>
    [
        new("空投总质量", "t"), new("Z向转动惯量", "kg.m^2"), new("X向转动惯量", "kg.m^2"), new("质心X向坐标", "m"), new("质心Z向坐标", "m"),
        new("空降车长度", "m"), new("空降车宽度", "m"), new("空降车高度", "m"), new("吊钩与质心垂直距离", "m"), new("吊钩与吊点垂直距离", "m"),
        new("吊钩与质心水平距离", "m"), new("牵引伞牵引点X坐标", "m"), new("牵引伞牵引点Z坐标", "m"), new("下支履带前端X坐标", "m"), new("下支履带前端Z坐标", "m"),
        new("下支履带后端X坐标", "m"), new("下支履带后端Z坐标", "m"), new("车辆空气阻力系数", ""), new("滑动摩擦系数", "")
    ];
    private static ParamDef[] PilotDefs() => [new("牵引伞质量", "kg"), new("牵引伞阻力系数", ""), new("牵引伞特征面积", "m^2"), new("牵引伞伞绳长度", "m")];
    private static ParamDef[] DecelerationDefs() => [new("减速伞伞衣质量", "kg"), new("减速伞阻力系数", ""), new("减速伞等效面积", "m^2"), new("减速伞伞边宽度", "m"), new("减速伞伞边质量", "kg"), new("减速伞伞绳数量", "个"), new("减速伞伞绳长度", "m"), new("减速伞伞绳质量", "kg"), new("减速伞附加质量系数", ""), new("减速伞充气系数", ""), new("减速伞开伞充气指数", "")];
    private static ParamDef[] MainChuteDefs() => [new("主伞特征面积", "m^2"), new("主伞数量", "具"), new("主伞伞衣质量", "kg"), new("主伞伞边宽度", "m"), new("主伞伞边质量", "kg"), new("主伞法向阻力系数", ""), new("主伞周向阻力系数", ""), new("主伞径向阻力系数", ""), new("主伞伞衣套阻力系数", ""), new("主伞已拉出物体阻力系数", ""), new("主伞拉出阻力", "N"), new("主伞伞绳数量", "个"), new("主伞伞绳质量", "kg"), new("主伞伞绳长度", "m"), new("主伞附加质量系数", ""), new("主伞收口比", ""), new("主伞第一次充气系数", ""), new("主伞第二次充气系数", ""), new("两次充气间隔时间", "s"), new("主伞延迟开伞时间", "s")];

    private void ShowConfigDialog()
    {
        var win = new Window { Title = "软件配置", Owner = this, Width = 760, Height = 260, WindowStartupLocation = WindowStartupLocation.CenterOwner };
        var grid = new Grid { Margin = new Thickness(22) };
        AddCols(grid, new GridLength(96), new GridLength(1, GridUnitType.Star));
        AddRows(grid, GridLength.Auto, GridLength.Auto, GridLength.Auto, GridLength.Auto);
        var data = Box(DataDir);
        var py = Box(PythonCommand);
        var script = Box(ScriptPath);
        AddLabelBox(grid, "数据目录", data, 0);
        AddLabelBox(grid, "Python", py, 1);
        AddLabelBox(grid, "脚本", script, 2);
        var actions = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Center, Margin = new Thickness(0, 18, 0, 0) };
        actions.Children.Add(Button("保存", (_, _) =>
        {
            if (_dataDirBox is not null) _dataDirBox.Text = data.Text;
            if (_pythonBox is not null) _pythonBox.Text = py.Text;
            if (_scriptBox is not null) _scriptBox.Text = script.Text;
            win.DialogResult = true;
        }, 100, 34));
        actions.Children.Add(Button("取消", (_, _) => win.Close(), 100, 34));
        Grid.SetRow(actions, 3); Grid.SetColumnSpan(actions, 2);
        grid.Children.Add(actions);
        win.Content = grid;
        win.ShowDialog();
    }

    private static GroupBox Section(string title, UIElement content, double? height)
    {
        var group = new GroupBox
        {
            Header = title,
            Content = content,
            Margin = new Thickness(6),
            Padding = new Thickness(8),
            Background = new SolidColorBrush(Color.FromRgb(245, 246, 248)),
            BorderBrush = BorderBrush()
        };
        if (height.HasValue) group.Height = height.Value;
        return group;
    }

    private static void AddInput(Grid grid, string label, TextBox box, string unit, int row, int col)
    {
        var labelBlock = Text(label, 12);
        Grid.SetRow(labelBlock, row); Grid.SetColumn(labelBlock, col);
        grid.Children.Add(labelBlock);
        Grid.SetRow(box, row); Grid.SetColumn(box, col + 1);
        box.Margin = new Thickness(0, row == 0 ? 0 : 7, 8, 0);
        grid.Children.Add(box);
        var unitBlock = Text(unit, 12);
        unitBlock.Margin = new Thickness(4, row == 0 ? 0 : 7, 0, 0);
        Grid.SetRow(unitBlock, row); Grid.SetColumn(unitBlock, col + 2);
        grid.Children.Add(unitBlock);
    }

    private static void AddLabelBox(Grid grid, string label, TextBox box, int row)
    {
        var labelBlock = Text(label, 12);
        labelBlock.Margin = new Thickness(0, row == 0 ? 0 : 7, 6, 0);
        Grid.SetRow(labelBlock, row); Grid.SetColumn(labelBlock, 0);
        grid.Children.Add(labelBlock);
        box.Margin = new Thickness(0, row == 0 ? 0 : 7, 0, 0);
        Grid.SetRow(box, row); Grid.SetColumn(box, 1);
        grid.Children.Add(box);
    }

    private static TextBox Box(string text, bool readOnly = false) => new()
    {
        Text = text,
        Height = 26,
        IsReadOnly = readOnly,
        VerticalContentAlignment = VerticalAlignment.Center
    };

    private static Button Button(string text, RoutedEventHandler? handler = null, double width = 86, double height = 30)
    {
        var button = new Button { Content = text, Width = width, Height = height, MinWidth = width, Margin = new Thickness(4), Padding = new Thickness(8, 0, 8, 0) };
        if (handler is not null) button.Click += handler;
        return button;
    }

    private static Button HomeButton(string text, RoutedEventHandler handler)
    {
        var button = Button(text, handler, 190, 96);
        button.FontSize = 18;
        button.FontWeight = FontWeights.SemiBold;
        button.Foreground = new SolidColorBrush(Color.FromRgb(23, 61, 85));
        button.Background = new SolidColorBrush(Color.FromRgb(221, 245, 244));
        button.BorderBrush = new SolidColorBrush(Color.FromRgb(182, 227, 227));
        button.Clip = Geometry.Parse("M 24,0 L 166,0 L 190,48 L 166,96 L 24,96 L 0,48 Z");
        return button;
    }

    private static TextBlock Text(string text, double size, FontWeight? weight = null, Brush? brush = null, HorizontalAlignment align = HorizontalAlignment.Left, Thickness? margin = null) => new()
    {
        Text = text,
        FontSize = size,
        FontWeight = weight ?? FontWeights.Normal,
        Foreground = brush ?? Brushes.Black,
        HorizontalAlignment = align,
        VerticalAlignment = VerticalAlignment.Center,
        Margin = margin ?? new Thickness(0)
    };

    private static TextBlock CenterText(string text) => new()
    {
        Text = text,
        HorizontalAlignment = HorizontalAlignment.Center,
        VerticalAlignment = VerticalAlignment.Center,
        Foreground = new SolidColorBrush(Color.FromRgb(102, 112, 133))
    };

    private static System.Windows.Shapes.Path ShapePath(string data, string color) => new()
    {
        Data = Geometry.Parse(data),
        Fill = (Brush)new BrushConverter().ConvertFromString(color)!,
        Stretch = Stretch.None
    };

    private static Brush TryBackground(string fileName, Brush fallback)
    {
        foreach (string path in BackgroundCandidates(fileName))
        {
            if (!File.Exists(path)) continue;
            var image = new BitmapImage();
            image.BeginInit();
            image.CacheOption = BitmapCacheOption.OnLoad;
            image.UriSource = new Uri(path, UriKind.Absolute);
            image.EndInit();
            return new ImageBrush(image) { Stretch = Stretch.UniformToFill };
        }
        return fallback;
    }

    private static IEnumerable<string> BackgroundCandidates(string fileName)
    {
        yield return IoPath.Combine(AppContext.BaseDirectory, "Assets", "Images", fileName);
        var directory = new DirectoryInfo(AppContext.BaseDirectory);
        while (directory is not null)
        {
            yield return IoPath.Combine(directory.FullName, "csharp_frontend", "Assets", "Images", fileName);
            yield return IoPath.Combine(directory.FullName, "Assets", "Images", fileName);
            directory = directory.Parent;
        }
    }

    private static Brush SkyBrush() => new LinearGradientBrush(
        [new GradientStop(Color.FromRgb(12, 95, 168), 0), new GradientStop(Color.FromRgb(165, 223, 240), 0.55), new GradientStop(Color.FromRgb(234, 247, 248), 1)],
        new Point(0, 0),
        new Point(1, 1));

    private static Brush HomeBrush() => new LinearGradientBrush(
        [new GradientStop(Color.FromRgb(217, 247, 244), 0), new GradientStop(Color.FromRgb(45, 146, 196), 0.55), new GradientStop(Color.FromRgb(4, 80, 168), 1)],
        new Point(0, 0),
        new Point(1, 1));

    private static SolidColorBrush BorderBrush() => new(Color.FromRgb(191, 199, 210));

    private static void AddRows(Grid grid, params GridLength[] heights)
    {
        foreach (var height in heights) grid.RowDefinitions.Add(new RowDefinition { Height = height });
    }

    private static void AddCols(Grid grid, params GridLength[] widths)
    {
        foreach (var width in widths) grid.ColumnDefinitions.Add(new ColumnDefinition { Width = width });
    }

    private static double ParseBox(TextBox? box, string name)
    {
        string text = box?.Text.Trim() ?? "";
        if (!double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out double value))
            throw new InvalidOperationException($"{name} 不是有效数值：{text}");
        return value;
    }

    private void SetStatus(string message)
    {
        if (_statusBlock is not null) _statusBlock.Text = message;
    }

    private void SetBusy(bool busy, string? message = null)
    {
        if (_runFirstButton is not null) _runFirstButton.IsEnabled = !busy;
        if (_runSecondButton is not null) _runSecondButton.IsEnabled = !busy;
        if (_runAllButton is not null) _runAllButton.IsEnabled = !busy;
        if (message is not null) SetStatus(message);
    }

    private static string FindPythonBridgeScript()
    {
        var directory = new DirectoryInfo(AppContext.BaseDirectory);
        while (directory is not null)
        {
            string candidate = IoPath.Combine(directory.FullName, "2", "run_excel_bridge.py");
            if (File.Exists(candidate)) return candidate;
            directory = directory.Parent;
        }
        return IoPath.GetFullPath(IoPath.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "2", "run_excel_bridge.py"));
    }

    private readonly record struct ParamDef(string Name, string Unit);
}









