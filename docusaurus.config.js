// @ts-check
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Building AI-Driven Humanoid Robots with ROS 2, Simulation, and VLA Systems',
  favicon: 'img/favicon.ico',

  // ✅ Vercel root URL
  url: 'https://your-project-name.vercel.app',
  baseUrl: '/',   // ✅ MUST be '/' for Vercel

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          path: 'docs/docs',
          sidebarPath: './sidebars.js',
          editUrl:
            'https://github.com/abasitbaloch/Next-Gen-Humanoid-Robotics-Book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.png',
        href: '/docs/intro',   // ✅ fixed
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        
        // --- NEW: TRANSLATOR BUTTON (Fixed for Twin Iframes) ---
        {
          type: 'html',
          position: 'right', 
          // This button specifically targets the iframe with "mode=translator" in the URL
          value: `
            <button 
              onclick="const frame = document.querySelector('iframe[src*=\\'mode=translator\\']'); if(frame) frame.contentWindow.postMessage('open-translator', '*');" 
              style="background:none; border:none; cursor:pointer; font-size:1.5rem; display:flex; align-items:center; padding:0 10px; transition: transform 0.2s;" 
              onmouseover="this.style.transform='scale(1.1)'" 
              onmouseout="this.style.transform='scale(1)'"
              title="Open Translator"
            >
              🌐
            </button>
          `,
        },
        {
          href: 'https://github.com/abasitbaloch/Next-Gen-Humanoid-Robotics-Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            { label: 'Module 1: ROS 2', to: '/docs/module-1-ros2/intro' },
            { label: 'Module 2: Digital Twin', to: '/docs/module-2-digital-twin/intro' },
            { label: 'Module 3: AI-Robot Brain', to: '/docs/module-3-ai-brain/intro' },
            { label: 'Module 4: VLA', to: '/docs/module-4-vla/intro' },
            { label: 'Capstone: Autonomous Humanoid', to: '/docs/capstone/intro' },
          ],
        },
      ],
      copyright:
        `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },

  staticDirectories: ['docs/static', 'static'],
};

export default config;