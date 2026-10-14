export function MeasuredPhoto({ src, alt, bounds }: { src: string; alt: string; bounds?: number[] }) {
  return <div className="measured-photo">
    <img src={src} alt={alt} />
    {bounds && <svg viewBox="0 0 1000 1000" preserveAspectRatio="none" aria-hidden="true">
      <rect x={bounds[0]} y={bounds[1]} width={bounds[2] - bounds[0]} height={bounds[3] - bounds[1]}
        fill="none" stroke="#00e676" strokeWidth="3" vectorEffect="non-scaling-stroke" />
    </svg>}
  </div>;
}
