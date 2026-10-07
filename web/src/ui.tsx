import { forwardRef, type ButtonHTMLAttributes } from 'react';

// Selected shadcn-style button variant; no starter theme or utility framework.
export const Button = forwardRef<HTMLButtonElement, ButtonHTMLAttributes<HTMLButtonElement> & {
  variant?: 'default' | 'outline' | 'ghost' | 'danger';
}>(({ variant = 'outline', className = '', ...props }, ref) =>
  <button ref={ref} className={`button button-${variant} ${className}`} {...props} />);
