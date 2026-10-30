import { BrowserRouter, Route, Routes } from "react-router-dom";
import Layout from "./components/Layout";
import { ToastProvider } from "./components/Toast";
import { AuthProvider, useAuth } from "./hooks/useAuth";
import { RobotProvider } from "./hooks/useRobot";
import Camera from "./pages/Camera";
import Detection from "./pages/Detection";
import Hardware from "./pages/Hardware";
import Login from "./pages/Login";
import Logs from "./pages/Logs";
import Mapping from "./pages/Mapping";
import Maps from "./pages/Maps";
import Navigation from "./pages/Navigation";
import Overview from "./pages/Overview";
import Sensors from "./pages/Sensors";
import Settings from "./pages/Settings";
import Sorting from "./pages/Sorting";
import Telemetry from "./pages/Telemetry";

function Gate() {
  const { user, ready } = useAuth();
  if (!ready) return null;
  if (!user) return <Login />;
  return (
    <RobotProvider>
      <Routes>
        <Route element={<Layout />}>
          <Route index element={<Overview />} />
          <Route path="camera" element={<Camera />} />
          <Route path="mapping" element={<Mapping />} />
          <Route path="navigation" element={<Navigation />} />
          <Route path="detection" element={<Detection />} />
          <Route path="sorting" element={<Sorting />} />
          <Route path="telemetry" element={<Telemetry />} />
          <Route path="hardware" element={<Hardware />} />
          <Route path="sensors" element={<Sensors />} />
          <Route path="maps" element={<Maps />} />
          <Route path="logs" element={<Logs />} />
          <Route path="settings" element={<Settings />} />
          <Route path="*" element={<Overview />} />
        </Route>
      </Routes>
    </RobotProvider>
  );
}

export default function App() {
  return (
    <BrowserRouter>
      <ToastProvider>
        <AuthProvider>
          <Gate />
        </AuthProvider>
      </ToastProvider>
    </BrowserRouter>
  );
}
