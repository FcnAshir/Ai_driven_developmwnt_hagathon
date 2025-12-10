# Railway Deployment Guide for ChatBot Backend

This guide will help you deploy your backend to Railway so it can be accessed from anywhere.

## Prerequisites

1. **Sign up for Railway**: Go to [railway.app](https://railway.app) and create an account
2. **Install Railway CLI** (optional): `npm install -g @railway/cli`
3. **Your backend code** (already prepared in the `backend` folder)

## Step-by-Step Deployment

### 1. Prepare Your Code (Already Done)

Your backend is already configured properly for Railway deployment:
- ✅ Uses `PORT` environment variable (line 139 in `src/main.py`)
- ✅ Binds to `0.0.0.0` (line 138 in `src/main.py`)
- ✅ CORS is enabled with wildcard (`access-control-allow-origin: *`)
- ✅ Proper requirements.txt file exists
- ✅ Uses environment variables for configuration

### 2. Create a Procfile

Create a file named `Procfile` (no extension) in your `backend` folder:

```
web: uvicorn src.main:app --host=0.0.0.0 --port=${PORT:-8000}
```

### 3. Deploy to Railway

#### Option A: Using Git and Railway Dashboard

1. **Initialize Git** in your project:
   ```bash
   cd D:\Ai_driven_developmwnt_hagathon
   git init
   git add .
   git commit -m "Initial commit for Railway deployment"
   ```

2. **Push to GitHub** (create a new repository):
   - Go to [github.com](https://github.com)
   - Create a new repository
   - Push your code:
   ```bash
   git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPOSITORY.git
   git branch -M main
   git push -u origin main
   ```

3. **Deploy on Railway**:
   - Go to [railway.app](https://railway.app)
   - Click "New Project"
   - Select "Deploy from GitHub"
   - Choose your repository
   - Click "Deploy Now"

#### Option B: Using Railway CLI

1. **Install Railway CLI**:
   ```bash
   npm install -g @railway/cli
   ```

2. **Login to Railway**:
   ```bash
   railway login
   ```

3. **Navigate to your backend folder**:
   ```bash
   cd D:\Ai_driven_developmwnt_hagathon\backend
   ```

4. **Initialize Railway project**:
   ```bash
   railway init
   ```

5. **Deploy**:
   ```bash
   railway up
   ```

### 4. Configure Environment Variables

After deployment, you need to set up the required environment variables in Railway:

1. Go to your Railway project dashboard
2. Click on "Variables" tab
3. Add the following variables:

```
GEMINI_API_KEY=your_actual_gemini_api_key_here
JWT_SECRET=your_secure_jwt_secret_here
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=postgresql://username:password@host:port/database_name
DEBUG=false
```

### 5. Update Your Frontend

Once deployed, your backend will have a URL like:
`https://your-project-name-production.up.railway.app`

Update your chatbot HTML files to use this URL instead of `http://localhost:8000`:

```javascript
// Change from:
const BACKEND_URL = 'http://localhost:8000';

// To:
const BACKEND_URL = 'https://your-project-name-production.up.railway.app';
```

## Required Environment Variables

### Gemini API Key
- Get from: [Google AI Studio](https://aistudio.google.com/)
- Variable name: `GEMINI_API_KEY`

### JWT Secret
- Generate a secure secret (32+ random characters)
- Variable name: `JWT_SECRET`

### Qdrant Configuration (for RAG)
- Get from: [Qdrant Cloud](https://qdrant.tech/)
- Variables: `QDRANT_URL` and `QDRANT_API_KEY`

### Database URL (if using PostgreSQL)
- From Railway's PostgreSQL plugin or external provider
- Variable name: `DATABASE_URL`

## Testing Your Deployment

1. **Check the health endpoint**:
   ```
   https://your-project-name-production.up.railway.app/.well-known/health
   ```

2. **Test the chat API**:
   ```
   POST https://your-project-name-production.up.railway.app/chatkit-public
   ```

## Troubleshooting

### Common Issues:

1. **Application crashes after deployment**:
   - Check Railway logs for error messages
   - Ensure all required environment variables are set

2. **API returns 500 errors**:
   - Verify GEMINI_API_KEY is correctly set
   - Check Qdrant connection details

3. **CORS issues**:
   - Your backend already has CORS enabled with wildcard
   - Should not be an issue

### Useful Commands:

```bash
# View logs
railway logs

# Check environment variables
railway variables

# Redeploy
railway up
```

## Security Notes

- Never commit API keys to Git
- Use strong, unique values for JWT_SECRET
- Enable HTTPS (Railway provides this automatically)
- Consider rate limiting for production use

## Next Steps

1. Deploy to Railway using the steps above
2. Update your frontend to use the Railway URL
3. Test the connection
4. Share your working chatbot with others!

Your backend is ready for deployment! The code is properly configured and should work without any modifications once deployed to Railway.