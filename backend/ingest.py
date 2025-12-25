import os
import requests
from bs4 import BeautifulSoup
import xml.etree.ElementTree as ET
from qdrant_client import QdrantClient
from qdrant_client.http import models
from fastembed import TextEmbedding
import logging
from urllib.parse import urlparse, urlunparse
import time
from dotenv import load_dotenv

# Force load the .env file
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '.env'))

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def load_environment_variables():
    """Load and validate environment variables."""
    required_vars = ['SITE_URL', 'QDRANT_URL', 'QDRANT_API_KEY']
    env_vars = {}
    for var in required_vars:
        value = os.getenv(var)
        if not value:
            raise ValueError(f"Missing required environment variable: {var}")
        env_vars[var] = value
    return env_vars

def fix_url(bad_url, correct_base_url):
    """
    Fixes URLs that point to 'your-project-name' by replacing 
    the domain with the correct SITE_URL domain.
    """
    parsed_bad = urlparse(bad_url)
    parsed_correct = urlparse(correct_base_url)
    
    # Force the URL to use the correct scheme (https) and domain (your vercel app)
    new_url = parsed_bad._replace(scheme=parsed_correct.scheme, netloc=parsed_correct.netloc).geturl()
    return new_url

def fetch_sitemap(site_url):
    if site_url.endswith('sitemap.xml'):
        sitemap_url = site_url
    else:
        sitemap_url = site_url.rstrip('/') + "/sitemap.xml"
        
    logger.info(f"Fetching sitemap from: {sitemap_url}")
    response = requests.get(sitemap_url)
    response.raise_for_status()
    return response.content

def parse_sitemap(sitemap_content, real_site_url):
    """Parse sitemap and FIX the URLs to match the real site."""
    root = ET.fromstring(sitemap_content)
    
    # Namespaces usually found in sitemaps
    namespaces = {
        'd': 'http://www.sitemaps.org/schemas/sitemap/0.9',
        'n': 'http://www.google.com/schemas/sitemap-news/0.9'
    }
    
    urls = []
    # Find all locations
    for loc in root.findall('.//d:loc', namespaces) + root.findall('.//loc'):
        if loc.text:
            # --- THE FIX HAPPENS HERE ---
            # We take the URL from the sitemap, but force it to use YOUR real domain
            corrected_url = fix_url(loc.text, real_site_url)
            urls.append(corrected_url)

    unique_urls = list(dict.fromkeys(urls))
    logger.info(f"Found {len(unique_urls)} unique URLs (Auto-corrected to match your site)")
    return unique_urls

def scrape_page_content(url):
    try:
        logger.info(f"Scraping: {url}")
        response = requests.get(url)
        response.raise_for_status() # This will fail if the URL is still wrong

        soup = BeautifulSoup(response.content, 'html.parser')
        
        # Remove junk
        for script in soup(["script", "style", "nav", "footer", "header", "aside"]):
            script.decompose()

        text = soup.get_text()
        
        # Clean text
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)
        
        return text
    except Exception as e:
        logger.error(f"Error scraping {url}: {e}")
        return ""

def main():
    logger.info("Starting book ingestion process...")
    
    # 1. Load Config
    try:
        env_vars = load_environment_variables()
    except Exception as e:
        logger.error(f"Configuration Error: {e}")
        return

    site_url = env_vars['SITE_URL']
    
    # 2. Setup Qdrant
    client = QdrantClient(
        url=env_vars['QDRANT_URL'], 
        api_key=env_vars['QDRANT_API_KEY']
    )
    
    # 3. Setup Brain
    logger.info("🧠 Loading Embedding Model...")
    embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
    
    # 4. Get URLs
    try:
        xml_content = fetch_sitemap(site_url)
        # Pass the REAL site URL so we can fix the bad ones
        urls = parse_sitemap(xml_content, site_url) 
    except Exception as e:
        logger.error(f"Failed to process sitemap: {e}")
        return

    # 5. Reset Collection (Optional: Clean slate)
    collection_name = "book_knowledge"
    try:
        client.delete_collection(collection_name)
        logger.info("🗑️ Cleared old empty collection.")
    except:
        pass
        
    client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE)
    )

    # 6. Scrape & Upload
    total_points = 0
    for i, url in enumerate(urls):
        logger.info(f"Processing ({i+1}/{len(urls)})")
        
        content = scrape_page_content(url)
        if not content:
            continue
            
        # Chunking (Splitting long text into small pieces)
        chunk_size = 1000
        text_chunks = [content[i:i+chunk_size] for i in range(0, len(content), chunk_size)]
        
        points = []
        for chunk in text_chunks:
            # Turn text into numbers
            vector = list(embedding_model.embed([chunk]))[0].tolist()
            
            points.append(models.PointStruct(
                id=total_points,
                vector=vector,
                payload={"text": chunk, "source_url": url}
            ))
            total_points += 1
            
        if points:
            client.upsert(collection_name=collection_name, points=points)
            logger.info(f"✅ Uploaded {len(points)} chunks from this page.")
            
        time.sleep(0.2) # Be nice to the server

    logger.info(f"🎉 SUCCESS! Ingested {total_points} chunks of knowledge.")

if __name__ == "__main__":
    main()