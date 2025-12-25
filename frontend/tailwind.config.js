/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        chat: {
          dark: "#0f172a",   // Deep blue/black background
          bubble: "#1e293b", // Message bubble color
          accent: "#3b82f6", // Bright blue for user messages
        }
      }
    },
  },
  plugins: [],
}