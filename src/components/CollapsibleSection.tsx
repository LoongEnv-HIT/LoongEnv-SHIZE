import React from 'react';
import { ChevronRight } from 'lucide-react';
import { cn } from '../lib/utils';

interface CollapsibleSectionProps {
  title: string;
  children: React.ReactNode;
  defaultOpen?: boolean;
  className?: string;
  contentClassName?: string;
}

export function CollapsibleSection({
  title,
  children,
  defaultOpen = true,
  className,
  contentClassName,
}: CollapsibleSectionProps) {
  return (
    <details
      open={defaultOpen}
      className={cn(
        "group/details overflow-hidden rounded-sm border border-[#e5e5e5] bg-white shadow-sm",
        className
      )}
    >
      <summary className="flex h-8 list-none items-center gap-1.5 bg-[#f3f3f3] px-2 text-[#333333] cursor-pointer transition-colors hover:bg-[#e8e8e8]">
        <ChevronRight className="h-4 w-4 text-[#6f6f6f] transition-transform group-open/details:rotate-90" />
        <span className="text-[10px] font-bold uppercase tracking-wider text-[#4b5563]">{title}</span>
      </summary>
      <div className={cn("border-t border-[#ececec] bg-white p-3.5", contentClassName)}>
        {children}
      </div>
    </details>
  );
}
