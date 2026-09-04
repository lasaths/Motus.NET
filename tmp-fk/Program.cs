using Motus.Core;
using Motus.Geometry;
using Motus.Presets;

var path = @"C:\Users\lasaths\GitHub\Motus.Grasshopper\resources\robots\ur10e_robotiq\ur10e_robotiq.urdf";
var bundle = UrdfRobotLoader.Load(path, new UrdfLoadOptions { BaseLink = "base_link", TipLink = "tool0", ModelName = "ur10e_robotiq" });
var fk = KinematicsResolver.CreateFkSolver(bundle.ToModel().Preset, bundle.Chain);
var robotiqTcp = new Frame(0, 0, 0.1633, 0.7071067811865476, 0, 0.7071067811865476, 0);
var tool = new ToolFrame(robotiqTcp, "robotiq");
void Dump(string name, double[] q) {
  var tcp = fk.ComputeTcp(new JointState(q), bundle.ToModel().Preset.BaseFrame, tool).Tcp;
  var m = Transforms.FromFrame(tcp);
  Console.WriteLine($"{name}: p=({tcp.X:F3},{tcp.Y:F3},{tcp.Z:F3}) MotusX=({m[0]:F3},{m[4]:F3},{m[8]:F3}) MotusY=({m[1]:F3},{m[5]:F3},{m[9]:F3}) MotusZ=({m[2]:F3},{m[6]:F3},{m[10]:F3})");
}
Dump("home0", new[]{0.0,-Math.PI/2,Math.PI/2,-Math.PI/2,Math.PI/2,0.0});
Dump("homeJ6pi", new[]{0.0,-Math.PI/2,Math.PI/2,-Math.PI/2,Math.PI/2,Math.PI});
Dump("flipW", new[]{0.0,-Math.PI/2,Math.PI/2,-Math.PI/2,-Math.PI/2,0.0});
Dump("flipW2", new[]{0.0,-Math.PI/2,Math.PI/2,Math.PI/2,Math.PI/2,0.0});
