import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

// CSS modules import kiya gaya hai
import styles from './index.module.css';

// Header section jismein title, tagline aur button hai
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    // 'hero hero--primary' Docusaurus ki default classes hain.
    // 'styles.heroBanner' custom styling ke liye hai (jo header ka green background set karega).
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            // Yeh button aapke custom CSS mein styled hoga (blue color, border-radius).
            className="button button--secondary button--lg"
            to="/docs/intro">
            Docusaurus Tutorial - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

// Main Home Page Component
export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        {/* Yeh section aapke Easy to Use, Focus on What Matters, Powered by React cards ko display karega */}
        <HomepageFeatures />
      </main>
    </Layout>
  );
}