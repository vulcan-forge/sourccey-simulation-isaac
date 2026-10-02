using System;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Windows.Forms;

internal static class Launcher {
    [STAThread]
    private static void Main() {
        bool first;
        using (var mutex = new Mutex(true, "Local\\SourcceyIsaacDesktop", out first)) {
            if (!first) {
                MessageBox.Show("Sourccey Isaac is already starting or running. Check its existing window.", "Sourccey Isaac");
                return;
            }
            string root = AppDomain.CurrentDomain.BaseDirectory;
            string logPath = Path.Combine(root, "logs", "latest.log");
            try {
                if (!File.Exists(Path.Combine(root, ".runtime", "python.bat")))
                    throw new InvalidOperationException("Run setup.cmd first to install NVIDIA Isaac Sim.");
                Directory.CreateDirectory(Path.GetDirectoryName(logPath));
                using (var log = new StreamWriter(logPath, false)) {
                    log.AutoFlush = true;
                    var info = new ProcessStartInfo("cmd.exe", "/d /c \"\"" + Path.Combine(root, "run.cmd") + "\"\"");
                    info.WorkingDirectory = root;
                    info.UseShellExecute = false;
                    info.CreateNoWindow = true;
                    info.RedirectStandardOutput = true;
                    info.RedirectStandardError = true;
                    using (var process = new Process()) {
                        process.StartInfo = info;
                        DataReceivedEventHandler record = (sender, line) => {
                            if (line.Data != null) lock (log) { log.WriteLine(line.Data); }
                        };
                        process.OutputDataReceived += record;
                        process.ErrorDataReceived += record;
                        process.Start();
                        process.BeginOutputReadLine();
                        process.BeginErrorReadLine();
                        process.WaitForExit();
                        if (process.ExitCode != 0)
                            throw new InvalidOperationException("Isaac Sim exited with code " + process.ExitCode + ".\nDetails: " + logPath);
                    }
                }
            } catch (Exception error) {
                MessageBox.Show(error.Message, "Sourccey Isaac - startup error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }
}
