// Operator console for the motion controller.
//
//   dotnet run -- [host] [port]                 interactive live view
//   dotnet run -- [host] [port] --set 1 90      send one setpoint and print status
//
// Commands in the live view: "<axis> <deg>", "all <deg>", "q".
using System.Globalization;
using System.Net.Sockets;
using System.Text;

var positional = args.TakeWhile(a => !a.StartsWith("--")).ToArray();
string host = positional.Length > 0 ? positional[0] : "127.0.0.1";
int port = positional.Length > 1 ? int.Parse(positional[1]) : 5555;
int setIdx = Array.IndexOf(args, "--set");

using var client = new TcpClient();
try { client.Connect(host, port); }
catch (SocketException e) { Console.Error.WriteLine($"Cannot reach controller at {host}:{port}: {e.Message}"); return 1; }

var stream = client.GetStream();
var reader = new StreamReader(stream, Encoding.ASCII);
var latest = new Status();
var connected = true;

void Send(int axis, double deg) =>
    stream.Write(Encoding.ASCII.GetBytes(string.Create(CultureInfo.InvariantCulture, $"SET {axis} {deg}\n")));

_ = Task.Run(() =>
{
    try
    {
        while (reader.ReadLine() is { } line)
            if (Status.TryParse(line, out var s)) latest = s;
    }
    catch (IOException) { }
    connected = false;
});

if (setIdx >= 0)  // one-shot mode, for scripts
{
    Send(int.Parse(args[setIdx + 1]), double.Parse(args[setIdx + 2], CultureInfo.InvariantCulture));
    Thread.Sleep(1500);
    Console.Write(latest.Render());
    return 0;
}

var input = new StringBuilder();
string message = "Commands: '<axis> <deg>', 'all <deg>', 'q'";
Console.Clear();
Console.CursorVisible = false;
while (connected)
{
    while (Console.KeyAvailable)
    {
        var key = Console.ReadKey(intercept: true);
        if (key.Key == ConsoleKey.Enter)
        {
            var parts = input.ToString().Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
            input.Clear();
            if (parts is ["q"]) { Console.CursorVisible = true; return 0; }
            if (parts.Length == 2 && double.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out var deg))
            {
                if (parts[0] == "all") { foreach (var ax in latest.Axes) Send(ax.Id, deg); message = $"All axes -> {deg} deg"; }
                else if (int.TryParse(parts[0], out var id)) { Send(id, deg); message = $"Axis {id} -> {deg} deg"; }
                else message = "Unknown command";
            }
            else message = "Usage: '<axis> <deg>', 'all <deg>', 'q'";
        }
        else if (key.Key == ConsoleKey.Backspace && input.Length > 0) input.Length--;
        else if (!char.IsControl(key.KeyChar)) input.Append(key.KeyChar);
    }
    Console.SetCursorPosition(0, 0);
    Console.Write(latest.Render());
    Console.WriteLine($"\n{message}".PadRight(70));
    Console.Write($"> {input}".PadRight(70));
    Console.SetCursorPosition(2 + input.Length, Console.CursorTop);
    Thread.Sleep(100);
}
Console.CursorVisible = true;
Console.WriteLine("\nController disconnected.");
return 0;

record Axis(int Id, double Setpoint, double Position, double Velocity, double Effort);

class Status
{
    public long Cycles;
    public long MaxLateUs;
    public List<Axis> Axes = new();

    // "STATUS <cycles> <max_late_us> <id>:<sp>:<pos>:<vel>:<effort> ..."
    public static bool TryParse(string line, out Status s)
    {
        s = new Status();
        var p = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (p.Length < 3 || p[0] != "STATUS") return false;
        s.Cycles = long.Parse(p[1]);
        s.MaxLateUs = long.Parse(p[2]);
        foreach (var a in p.Skip(3))
        {
            var f = a.Split(':');
            double D(int i) => double.Parse(f[i], CultureInfo.InvariantCulture);
            s.Axes.Add(new Axis(int.Parse(f[0]), D(1), D(2), D(3), D(4)));
        }
        return true;
    }

    public string Render()
    {
        var sb = new StringBuilder();
        sb.AppendLine("REAL-TIME MOTION CONTROL - OPERATOR".PadRight(70));
        sb.AppendLine($"control cycles {Cycles,10}   max loop latency {MaxLateUs,7} us".PadRight(70));
        sb.AppendLine();
        sb.AppendLine($"{"axis",4} {"setpoint",10} {"position",10} {"error",8} {"vel rad/s",10} {"torque Nm",10}");
        foreach (var a in Axes)
            sb.AppendLine(string.Create(CultureInfo.InvariantCulture,
                $"{a.Id,4} {a.Setpoint,10:F2} {a.Position,10:F2} {a.Setpoint - a.Position,8:F2} {a.Velocity,10:F3} {a.Effort,10:F3}").PadRight(70));
        return sb.ToString();
    }
}
