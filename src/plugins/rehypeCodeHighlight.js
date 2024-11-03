import { getHighlighter } from 'shiki'
import rehypeShiki from '@shikijs/rehype'

const highlighter = await getHighlighter({
  themes: ['github-light', 'github-dark'],
  langs: ['javascript', 'python', 'c', 'C++'] // 添加你需要的语言
})

export const rehypeCodeHighlight = [
  [rehypeShiki, { highlighter }]
]