# Deploying to Vercel

This Docusaurus site can be easily deployed to Vercel. Follow these steps:

## Option 1: Deploy via Vercel Dashboard (Recommended)

1. **Push your code to GitHub**
   ```bash
   git add .
   git commit -m "Prepare for Vercel deployment"
   git push
   ```

2. **Import Project on Vercel**
   - Go to [vercel.com](https://vercel.com)
   - Click "Add New Project"
   - Import your GitHub repository
   - Vercel will auto-detect Docusaurus settings

3. **Configure Build Settings** (if needed)
   - Build Command: `npm run build`
   - Output Directory: `build`
   - Install Command: `npm install`
   - Framework Preset: Other

4. **Deploy**
   - Click "Deploy"
   - Your site will be live at `https://your-project.vercel.app`

## Option 2: Deploy via Vercel CLI

1. **Install Vercel CLI**
   ```bash
   npm i -g vercel
   ```

2. **Login to Vercel**
   ```bash
   vercel login
   ```

3. **Deploy**
   ```bash
   vercel
   ```

4. **For production deployment**
   ```bash
   vercel --prod
   ```

## Important Notes

### Base URL Configuration

If you're deploying to a **custom domain** or want the site at the **root path**, update `docusaurus.config.js`:

```javascript
baseUrl: '/',  // Change from '/next-gen-humanoid-robotics-book/'
url: 'https://your-domain.com',  // Your Vercel domain or custom domain
```

If you're deploying to a **subpath** (like `vercel.app/next-gen-humanoid-robotics-book`), keep the current `baseUrl: '/next-gen-humanoid-robotics-book/'`.

### Environment Variables

No environment variables are required for basic deployment.

### Build Output

The `build/` directory contains the static files that Vercel will serve. This is already configured in `vercel.json`.

## Troubleshooting

- **404 errors**: Make sure `baseUrl` matches your deployment path
- **Build fails**: Check Node.js version (requires >= 18)
- **Assets not loading**: Verify `staticDirectories` in `docusaurus.config.js` includes `docs/static`

