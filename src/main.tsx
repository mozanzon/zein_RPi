import React from "react";
import ReactDOM from "react-dom/client";
import App from "./app/App";
import "./styles/fonts.css";
import "leaflet/dist/leaflet.css";
import "./styles/theme.css";
import "./styles/custom.css";
import "./styles/tailwind.css";

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
);
