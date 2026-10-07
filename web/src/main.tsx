import { createRoot } from 'react-dom/client';
import { App } from './App';
import '@radix-ui/colors/gray-dark.css';
import '@radix-ui/colors/orange-dark.css';
import '@radix-ui/colors/blue-dark.css';
import '@radix-ui/colors/cyan-dark.css';
import '@radix-ui/colors/yellow-dark.css';
import '@radix-ui/colors/red-dark.css';
import 'leaflet/dist/leaflet.css';
import 'gridstack/dist/gridstack.min.css';
import './style.css';

createRoot(document.getElementById('root')!).render(<App />);
