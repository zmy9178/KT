# WPF 与 Python 对接

## 1. 启动 Python 服务

在项目目录 `C:\Users\Tree\Desktop\2\2` 执行：

```powershell
py -3.13 api_server.py
```

服务地址：

```text
http://127.0.0.1:8765
```

Python 服务使用标准库 HTTP Server，不需要安装 FastAPI、Flask 等额外 Web 框架。

## 2. 接口

### 健康检查

```http
GET /api/health
```

返回：

```json
{"success":true,"status":"ok","service":"AirdropPythonBridge/1.0"}
```

### 读取默认参数

```http
GET /api/defaults
```

### 运行仿真

```http
POST /api/simulations
Content-Type: application/json
```

请求体可以省略参数，省略时使用 MATLAB 迁移代码中的默认值：

```json
{
  "kte_p": [0, 800, 320, 0, 5.75, -1.24, 6, 7, 25],
  "vel_p": [15.2, 49600, 26400, -3.279, -0.407, 5.6, 3.2, 2.2, 0.506, 4, 1.614, -0.34, -0.698, -1.209, -4.9, -5.75, -4.9, 0.83, 0.05],
  "qys_p": [20.2, 0.8, 14, 5.55],
  "jss_p": [10, 0.8, 6.8, 0.01, 2, 8, 22, 4, 1, 8, 0.9],
  "zhs_p": [760, 9, 58.767, 0.026, 6.618, 1, 0.8, 0.8, 0.8, 0.6, 200, 56, 24.734, 80.425, 1, 0.16, 23.5, 4.3, 5],
  "sim_p": [50, 0.001],
  "include_steady_descent": false,
  "steady_duration": 200
}
```

返回的 `data_all` 每一行对应 `columns`：

```text
x, height, lateral_displacement, x_speed, vertical_speed,
lateral_speed, pitch_angle, roll_angle, overload
```

## 3. WPF 调用类

将下面的类放到 WPF 项目中。项目需要引用 `System.Net.Http.Json`，通常 .NET 6/7/8 已自带。

```csharp
using System.Net.Http;
using System.Net.Http.Json;

public sealed record SimulationRequest(
    double[] KteP,
    double[] VelP,
    double[] QysP,
    double[] JssP,
    double[] ZhsP,
    double[] SimP,
    bool IncludeSteadyDescent = true,
    double SteadyDuration = 200);

public sealed class SimulationResponse
{
    public bool Success { get; set; }
    public double[] TimeAll { get; set; } = Array.Empty<double>();
    public double[][] DataAll { get; set; } = Array.Empty<double[]>();
    public string[] Columns { get; set; } = Array.Empty<string>();
    public Dictionary<string, double> StageTimes { get; set; } = new();
}

public sealed class PythonSimulationClient
{
    private readonly HttpClient _httpClient;

    public PythonSimulationClient(HttpClient httpClient)
    {
        _httpClient = httpClient;
        _httpClient.BaseAddress = new Uri("http://127.0.0.1:8765/");
        _httpClient.Timeout = TimeSpan.FromMinutes(10);
    }

    public async Task<bool> IsAvailableAsync(CancellationToken cancellationToken = default)
    {
        using var response = await _httpClient.GetAsync(
            "api/health", cancellationToken);
        return response.IsSuccessStatusCode;
    }

    public async Task<SimulationResponse> RunAsync(
        SimulationRequest request,
        CancellationToken cancellationToken = default)
    {
        using var response = await _httpClient.PostAsJsonAsync(
            "api/simulations", request, cancellationToken);
        response.EnsureSuccessStatusCode();

        var result = await response.Content.ReadFromJsonAsync<SimulationResponse>(
            cancellationToken);
        return result ?? throw new InvalidOperationException(
            "Python returned an empty response.");
    }
}
```

调用示例：

```csharp
private readonly PythonSimulationClient _python =
    new(new HttpClient());

private async void RunButton_Click(object sender, RoutedEventArgs e)
{
    try
    {
        RunButton.IsEnabled = false;
        var request = new SimulationRequest(
            KteP: new[] { 0d, 800d, 320d, 0d, 5.75d, -1.24d, 6d, 7d, 25d },
            VelP: GetVelocityParametersFromControls(),
            QysP: GetQysParametersFromControls(),
            JssP: GetJssParametersFromControls(),
            ZhsP: GetZhsParametersFromControls(),
            SimP: new[] { 50d, 0.001d },
            IncludeSteadyDescent: false);

        var result = await _python.RunAsync(request);
        // result.TimeAll 和 result.DataAll 可绑定到 DataGrid 或绘图控件。
    }
    catch (Exception ex)
    {
        MessageBox.Show(ex.Message, "仿真失败");
    }
    finally
    {
        RunButton.IsEnabled = true;
    }
}
```

## 4. 进程管理建议

WPF 发布时可由 C# 启动 Python 服务进程，并在窗口关闭时释放：

```csharp
using System.Diagnostics;

var process = Process.Start(new ProcessStartInfo
{
    FileName = "py",
    Arguments = "-3.13 api_server.py",
    WorkingDirectory = pythonDirectory,
    UseShellExecute = false,
    CreateNoWindow = true,
});
```

实际部署建议把 Python 项目用 PyInstaller 打包成 `airdrop-api.exe`，然后将
`FileName` 改成该 exe，避免目标电脑必须安装 Python。
