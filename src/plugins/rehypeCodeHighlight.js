import rehypeShiki from '@shikijs/rehype'
import { getHighlighter } from 'shiki'

export const rehypeCodeHighlight = async () => {
  const highlighter = await getHighlighter({
    themes: ['github-light', 'github-dark'],
    defaultTheme: 'github-dark',
  })

  return [
    rehypeShiki,
    { highlighter }
  ]
}
