%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Custom, stripped down version of egpubl.cls (EG Publications document style)
%
% based on EG Annual Event document style, in turn based on CFG Journal
% Copyright (c) 1997-2019 D.Fellner
%
% based on the original LaTeX ARTICLE DOCUMENT STYLE
%    for LaTeX version 2.09 by Leslie Lamport
% ...modifications by Sue Brooks, BUSS Ltd
% ... and Graham White 8.ix.92
% ...redesign for LaTeX2e and adaption to Conf. Proc. by D.Fellner
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\newif\ifoldfss
\newif\ifnfssone
\newif\ifnfsstwo
\def\f@s@s{}%
%
\@ifundefined{DeclareFontShape}%
  {%
   \@ifundefined{selectfont}%
    {\global\oldfsstrue\let\reset@font=\relax}% old font selection scheme
    {\global\nfssonetrue\def\f@s@s{with NFSS release 1,\space}}%
  }%
  {\global\nfsstwotrue\def\f@s@s{with NFSS release 2,\space}}%

\@twosidetrue
\@twocolumntrue
\@mparswitchtrue
\def\ds@draft{\overfullrule 5pt}
\def\ds@twocolumn{\@twocolumntrue}
\def\ds@onecolumn{\@twocolumnfalse}
\newif\ifSFB@galley
\def\ds@galley{\SFB@galleytrue\@twosidefalse\@mparswitchfalse}
\@options
%
%  ****************************************
%  *               FONTS                  *
%  ****************************************
%
\lineskip 1pt
\normallineskip 1pt
\def\baselinestretch{1}
  \def\@normalsize{\@setsize\normalsize{11pt}\ixpt\@ixpt
   \abovedisplayskip 5.5pt plus 2pt minus .5pt
   \belowdisplayskip \abovedisplayskip
   \abovedisplayshortskip 5.5pt plus 2pt
   \belowdisplayshortskip \abovedisplayshortskip
   \let\@listi\@listI}
\let\normalsize=\@normalsize
\def\small{\@setsize\small{10pt}\viiipt\@viiipt
 \abovedisplayskip 5pt plus 1pt minus .5pt
 \belowdisplayskip \abovedisplayskip
 \abovedisplayshortskip 5pt plus 1pt
 \belowdisplayshortskip \abovedisplayshortskip
 \def\@listi{\leftmargin\leftmargini
  \topsep  \z@ plus 1pt minus .5pt
  \parsep  \z@
  \itemsep \z@}}
\def\footnotesize{\@setsize\footnotesize{10pt}\viiipt\@viiipt
 \abovedisplayskip 5pt plus 1pt minus .5pt
 \belowdisplayskip \abovedisplayskip
 \abovedisplayshortskip 5pt plus 1pt
 \belowdisplayshortskip \abovedisplayshortskip
 \def\@listi{\leftmargin\leftmargini
  \topsep  \z@ plus 1pt minus .5pt
  \parsep  \z@
  \itemsep \z@}}
\def\scriptsize{\@setsize\scriptsize{7pt}\vipt\@vipt} % ajw: was 7/8
\def\tiny{\@setsize\tiny{8pt}\vipt\@vipt}
\def\large{\@setsize\large{11pt}\xpt\@xpt}
\def\Large{\@setsize\Large{14pt}\xiipt\@xiipt}
\def\LARGE{\@setsize\LARGE{17pt}\xivpt\@xivpt}
\def\huge{\@setsize\huge{20pt}\xviipt\@xviipt}
\def\Huge{\@setsize\Huge{25pt}\xxpt\@xxpt}
\normalsize

%  ****************************************
%  *            PAGE LAYOUT               *
%  ****************************************
%
% SIDE MARGINS:
\oddsidemargin = -23pt
\evensidemargin= -23pt
\marginparwidth 4pc
\marginparsep 0.5pc

% VERTICAL SPACING:
\topmargin -1pc
\headheight 11pt
\headsep    11pt
\topskip     9pt
\footskip   22pt

% DIMENSION OF TEXT:

\textwidth 42pc
\textheight = 57\baselineskip \advance\textheight by \topskip

\columnsep 2pc
\columnseprule 0pt

% FOOTNOTES:
\footnotesep 6pt
\skip\footins 19.5pt plus 12pt minus 1pt

% FLOATS:
% for floats on a text page:
%  one-column mode or single-column floats in two-column mode:
\floatsep     11pt plus 5.5pt minus 2.25pt
\textfloatsep 22pt plus 11pt minus 5.5pt
\intextsep    11pt plus 5.5pt minus 2.25pt
%    two-column floats in two-column mode:
\dblfloatsep     11pt plus 5.5pt minus 2.25pt
\dbltextfloatsep 22pt plus 11pt minus 5.5pt
% for floats on a separate float page or column:
% one-column mode or single-column floats in two-column mode:
\@fptop  0pt plus 0fil
\@fpsep 11pt plus 0.5fil
\@fpbot  0pt plus 3fil
% double-column floats in two-column mode.
\@dblfptop  0pt plus 0fil
\@dblfpsep 11pt plus 0.5fil
\@dblfpbot  0pt plus 3fil

\marginparpush 5.5pt

%  ****************************************
%  *           PARAGRAPHING               *
%  ****************************************
%
\parskip 5.5pt plus 1pt minus 1.5pt
\parindent 1em
\partopsep 0pt plus 1pt
\@lowpenalty   51
\@medpenalty  151
\@highpenalty 301
\@beginparpenalty -\@lowpenalty
\@endparpenalty -\@lowpenalty
\@itempenalty -\@lowpenalty
\clubpenalty=0
\widowpenalty=10000

%  ****************************************
%  *         FIGURES AND TABLES           *
%  ****************************************
%
\setcounter{topnumber}{2}
\def\topfraction{.9}
\setcounter{bottomnumber}{1}
\def\bottomfraction{.9}
\setcounter{totalnumber}{3}
\def\textfraction{.1}
\def\floatpagefraction{.75}
\setcounter{dbltopnumber}{2}
\def\dbltopfraction{.9}
\def\dblfloatpagefraction{.75}

\newcounter{table}
\def\thetable{\@arabic\c@table}
\ifSFB@galley \def\fps@table{p}
        \else \def\fps@table{tbp}
\fi
\def\ftype@table{1}
\def\ext@table{lot}
\def\fnum@table{Table \thetable}
\def\table{\@float{table}}
\let\endtable\end@float
\@namedef{table*}{\@dblfloat{table}}
\@namedef{endtable*}{\end@dblfloat}

\newcounter{algorithm}
\def\thealgorithm{\@arabic\c@algorithm}
\ifSFB@galley \def\fps@algorithm{p}
        \else \def\fps@algorithm{tbp}
\fi
\def\ftype@algorithm{2}
\def\ext@algorithm{lof}
\def\fnum@algorithm{Algorithm \thealgorithm}
\def\algorithm{\@float{algorithm}}
\let\endalgorithm\end@float

\newcounter{figure}
\def\thefigure{\@arabic\c@figure}
\ifSFB@galley \def\fps@figure{p}
        \else \def\fps@figure{tbp}
\fi
\def\ftype@figure{2}
\def\ext@figure{lof}
\def\fnum@figure{Figure \thefigure}
\def\figure{\@float{figure}}
\let\endfigure\end@float
\@namedef{figure*}{\@dblfloat{figure}}
\@namedef{endfigure*}{\end@dblfloat}

\long\def\@makecaption#1#2{\vskip 6pt
 \setbox\@tempboxa\hbox{\textbf{#1:} \reset@font\textit{#2}}
 \ifdim \wd\@tempboxa >\hsize
  \textbf{#1:} \reset@font\itshape #2\par
 \else
  \hbox to\hsize{\hfil\box\@tempboxa\hfil}
 \fi
 \vskip 6pt
}

%  ****************************************
%  *                PARTS                 *
%  ****************************************
%
\def\part{\par \addvspace{4ex}\@afterindentfalse
 \secdef\@part\@spart}
\def\@part[#1]#2{\ifnum \c@secnumdepth >\m@ne
  \refstepcounter{part}
  \addcontentsline{toc}{part}{Part \thepart: #1}
 \else \addcontentsline{toc}{part}{#1}
 \fi
 {\parindent 0pt \raggedright
  \ifnum \c@secnumdepth >\m@ne
   \reset@font\large\rmfamily PART
   \ifcase\thepart \or ONE \or TWO \or THREE \or FOUR \or FIVE
    \or SIX \or SEVEN \or EIGHT \or NINE \or TEN \else \fi
   \par \nobreak
  \fi
  \reset@font\LARGE \rmfamily #2 \markboth{}{}\par }
 \nobreak \vskip 3ex \@afterheading}
\def\@spart#1{{\parindent 0pt \raggedright
  \reset@font\LARGE \rmfamily #1\par}
 \nobreak \vskip 3ex \@afterheading}

%  ****************************************
%  *               SECTIONS               *
%  ****************************************
%
\def\section{\@startsection {section}{1}{\z@}
 {-11pt plus -5.5pt minus -1pt}
 {0.001pt}
 {\raggedright\reset@font\normalsize\bfseries}}
\def\subsection{\@startsection{subsection}{2}{\z@}
 {-11pt plus -5.5pt minus -1pt}
 {0.001pt}
 {\raggedright\reset@font\normalsize\bfseries}}
\def\subsubsection{\@startsection{subsubsection}{3}{\z@}
 {-5.5pt plus -5.5pt minus -1pt}
 {0.001pt}
 {\raggedright\reset@font\normalsize\bfseries}}
\def\paragraph{\@startsection{paragraph}{4}{\z@}
 {5.5pt plus 2.25pt minus 1pt}
 {-0.5em}
 {\reset@font\normalsize\bfseries}}
\def\subparagraph{\@startsection{subparagraph}{4}{1em}
 {5.5pt plus 2.25pt minus 1pt}
 {-0.5em}
 {\reset@font\normalsize\bfseries}}
\setcounter{secnumdepth}{5}

\def\@startsection#1#2#3#4#5#6{\if@noskipsec \leavevmode \fi
 \par \@tempskipa #4\relax
 \@afterindenttrue
 \ifdim \@tempskipa <\z@ \@tempskipa -\@tempskipa \@afterindentfalse\fi
 \if@nobreak \everypar{}\else
  \addpenalty{\@secpenalty}\addvspace{\@tempskipa}\fi
 \@ifstar{\SFB@ssect{#2}{#3}{#4}{#5}{#6}}%
         {\@dblarg{\@sect{#1}{#2}{#3}{#4}{#5}{#6}}}}
%
% \@sect{NAME}{LEVEL}{INDENT}{BEFORESKIP}{AFTERSKIP}{STYLE}[ARG1]{ARG2}
%
\def\@sect#1#2#3#4#5#6[#7]#8{%
 \ifnum #2>\c@secnumdepth
  \def\@svsec{}%
 \else
  \refstepcounter{#1}
  \edef\@svsec{\csname the#1\endcsname.\hskip 0.5em}%
 \fi
 \@tempskipa #5\relax
 \ifdim \@tempskipa>\z@
  \begingroup #6\relax
   \ifnum #2=\@ne
    \ifSFB@appendix
     \ifSFB@appendixstar
      \@hangfrom{\hskip #3\relax}{\interlinepenalty \@M
       Appendix \csname the#1\endcsname\par}%
     \else
      \@hangfrom{\hskip #3\relax}{\interlinepenalty \@M
       Appendix \csname the#1\endcsname:\hskip 0.5em\reset@font\rmfamily #8\par}%
     \fi
    \else
     \@hangfrom{\hskip #3\relax\@svsec}{\interlinepenalty \@M #8\par}%
    \fi
   \else
    \@hangfrom{\hskip #3\relax\@svsec}{\interlinepenalty \@M #8\par}%
   \fi
  \endgroup
  \csname #1mark\endcsname{#7}%
  \addcontentsline{toc}{#1}{\ifnum #2>\c@secnumdepth \else
   \protect\numberline{\csname the#1\endcsname}\fi #7}%
 \else
  \def\@svsechd{#6\hskip #3\@svsec #8%\par
   \csname #1mark\endcsname{#7}
   \addcontentsline{toc}{#1}{\ifnum #2>\c@secnumdepth \else
    \protect\numberline{\csname the#1\endcsname}\fi#7}}%
 \fi
 \@xsect{#5}}
%
% \SFB@ssect{LEVEL}{INDENT}{BEFORESKIP}{AFTERSKIP}{STYLE}{ARG} ==
%
\def\SFB@ssect#1#2#3#4#5#6{\@tempskipa #4\relax
 \ifSFB@appendix
  \ifnum #1>\@ne             \edef\@svsec{#6}%
   \else \ifSFB@appendixstar \edef\@svsec{Appendix}%
                       \else \edef\@svsec{Appendix:\hskip 0.5em\reset@font\rmfamily #6}%
   \fi
  \fi
 \else
  \ifnum #1>\@ne             \edef\@svsec{#6}%
           \else             \edef\@svsec{#6}%
  \fi
 \fi
 \ifdim \@tempskipa>\z@
  \begingroup #5\relax
    \@hangfrom{\hskip #2\relax}{\interlinepenalty \@M \@svsec \par}%
  \endgroup
 \else
  \def\@svsechd{#5\hskip #2\relax \@svsec}%
 \fi
 \@xsect{#4}}

\newif\ifSFB@appendix
\newif\ifSFB@appendixstar
\def\APPENDIX{\appendix\SFB@appendixstartrue}
\def\appendix{\SFB@appendixstarfalse\par
 \SFB@appendixtrue
 \setcounter{secnumdepth}{1}
 \setcounter{section}{0}
 \setcounter{subsection}{0}
 \def\thesection{\Alph{section}}}

%  ****************************************
%  *       PARTS & SECTIONS NUMBERS       *
%  ****************************************
%
\newcounter{part}
\newcounter {section}
\newcounter {subsection}[section]
\newcounter {subsubsection}[subsection]
\newcounter {paragraph}[subsubsection]
\newcounter {subparagraph}[paragraph]
\def\thepart          {\arabic{part}}
\def\thesection       {\arabic{section}}
\def\thesubsection    {\thesection.\arabic{subsection}}
\def\thesubsubsection {\thesubsection .\arabic{subsubsection}}
\def\theparagraph     {\thesubsubsection.\arabic{paragraph}}
\def\thesubparagraph  {\theparagraph.\arabic{subparagraph}}

%  ****************************************
%  *                LISTS                 *
%  ****************************************
%
\leftmargini   1em
\leftmarginii  1pc
\leftmarginiii 1pc
\leftmarginiv  1pc
\leftmarginv   1pc
\leftmarginvi  1pc
\leftmargin\leftmargini
\labelsep 0.5em
\labelwidth\leftmargini\advance\labelwidth-\labelsep
%
% label macros for Range-Left and Range-Right labels
\def\makeRLlabel#1{\rlap{#1}\hss}
\def\makeRRlabel#1{\hss\llap{#1}}
%
\def\@listI{\leftmargin\leftmargini
 \parsep  \z@
 \partopsep \z@
 \topsep  \z@ plus 1pt minus .5pt
 \itemsep \z@ plus .1pt
}
\let\@listi\@listI
\@listi
\def\@listii{\leftmargin\leftmarginii
 \labelwidth\leftmarginii\advance\labelwidth-\labelsep
 \topsep  5.5pt plus 1pt minus 1pt
 \parsep  \z@
 \partopsep \z@
 \itemsep \z@ plus .1pt
}
\def\@listiii{\leftmargin\leftmarginiii
 \labelwidth\leftmarginiii\advance\labelwidth-\labelsep
 \topsep 5.5pt plus 1pt minus 1pt
 \parsep \z@
 \partopsep \z@
 \itemsep \z@
}
\def\@listiv{\leftmargin\leftmarginiv
 \labelwidth\leftmarginiv\advance\labelwidth-\labelsep
}
\def\@listv{\leftmargin\leftmarginv
 \labelwidth\leftmarginv\advance\labelwidth-\labelsep
}
\def\@listvi{\leftmargin\leftmarginvi
 \labelwidth\leftmarginvi\advance\labelwidth-\labelsep
}
%
% ITEMIZE
\def\itemize{\ifnum \@itemdepth >3 \@toodeep
  \else \advance\@itemdepth \@ne
   \edef\@itemitem{labelitem\romannumeral\the\@itemdepth}%
   \list{\csname\@itemitem\endcsname}%
    {\let\makelabel\makeRLlabel}%
  \fi}
\def\labelitemi{$\bullet$}
\def\labelitemii{--}
\def\labelitemiii{$\circ$}
\def\labelitemiv{$\diamond$}
%
% ENUMERATE
% SFB -- enumerate with optional argument to set left margin...
\def\enumerate{\ifnum \@enumdepth >3 \@toodeep \else
  \advance\@enumdepth \@ne
  \edef\@enumctr{enum\romannumeral\the\@enumdepth}%
 \fi
 \@ifnextchar [{\@enumeratetwo}{\@enumerateone}%
}
\def\@enumeratetwo[#1]{\list{\csname label\@enumctr\endcsname}%
  {\settowidth\labelwidth{#1}
   \leftmargin\labelwidth \advance\leftmargin\labelsep
   \usecounter{\@enumctr}
   \let\makelabel\makeRRlabel}%
}
\def\@enumerateone{\list{\csname label\@enumctr\endcsname}%
  {\settowidth\labelwidth{9.}
   \leftmargin\labelwidth \advance\leftmargin\labelsep
   \usecounter{\@enumctr}
   \let\makelabel\makeRRlabel}%
}
\def\labelenumi{\theenumi}            \def\theenumi  {\arabic{enumi}.}
\def\labelenumii{\theenumii}          \def\theenumii {\alph{enumii}.}
\def\p@enumii{\theenumi}
\def\labelenumiii{\theenumiii}        \def\theenumiii{\roman{enumiii}.}
\def\p@enumiii{\theenumi(\theenumii)}
\def\labelenumiv{\theenumiv}          \def\theenumiv {\Alph{enumiv}.}
\def\p@enumiv{\p@enumiii\theenumiii}
%
% DESCRIPTION
\def\descriptionlabel#1{\hspace\labelsep \bfseries #1}
\def\description{\list{}{\labelwidth\z@ \itemindent-\leftmargin
  \let\makelabel\descriptionlabel}}
\let\enddescription\endlist

% TITLEPAGE
\def\titlepage{\@restonecolfalse\if@twocolumn\@restonecoltrue\onecolumn
  \else \newpage \fi \thispagestyle{empty}\c@page\z@}
\def\endtitlepage{\if@restonecol\twocolumn \else \newpage \fi}

% ARRAY AND TABULAR
%
\arraycolsep   5pt
\tabcolsep     6pt
\arrayrulewidth .5pt
\doublerulesep 1.5pt

\def\tabular{\let\@halignto\@empty
 \def\hline{\noalign{\ifnum0=`}\fi
  \hrule \@height \arrayrulewidth
  \futurelet \reserved@a \@xhline}
 \def\@xhline{\ifx\reserved@a\hline
   \vskip \doublerulesep \vskip -\arrayrulewidth
  \fi
  \ifnum0=`{\fi}}
  \def\@arrayrule{\@addtopreamble{\hskip -.5\arrayrulewidth
                                  \vrule \@width \arrayrulewidth
                                  \hskip .5\arrayrulewidth}}
 \let\fullhline\hline
\@tabular
}

% EQNARRAY
%
\def\eqnarray{\stepcounter{equation}%
 \let\@currentlabel=\theequation
 \global\@eqnswtrue
 \global\@eqcnt\z@
 \tabskip\@centering
 \let\\=\@eqncr
 $$\halign to \displaywidth\bgroup\@eqnsel\hskip\@centering
  $\displaystyle\tabskip\z@{##}$&\global\@eqcnt\@ne
  \hskip \arraycolsep
  \hfil${##}$\hfil
  &\global\@eqcnt\tw@
  \hskip \arraycolsep
  $\displaystyle\tabskip\z@{##}$\hfil
   \tabskip\@centering&\llap{##}\tabskip\z@\cr}

% TABBING
%
\tabbingsep \labelsep

% MINIPAGE
%
\skip\@mpfootins = \skip\footins

% FRAMEBOX
%
% Space left between box and text by \fbox and \framebox.
\fboxsep = 3pt
% Width of rules in box made by \fbox and \framebox.
\fboxrule = \arrayrulewidth

%  ****************************************
%  *         TITLE AND ABSTRACT           *
%  ****************************************
%
% added for teaser
\newcommand{\teaser@empty}{}

\newcommand{\@teaser}{}
\newcommand{\teaser}[1]{\renewcommand{\@teaser}{#1}}

\newlength{\titlespace}
\setlength{\titlespace}{-10pt}

\newlength{\teaserspace}
\setlength{\teaserspace}{12pt}


\def\maketitle{\par
 \begingroup
 \setcounter{footnote}{0}
  \def\thefootnote{\fnsymbol{footnote}}
  \def\@makefnmark{\mbox{$^{\@thefnmark}$}}
  \if@twocolumn
   \twocolumn[\vspace*{27pt}\@maketitle]
  \else
   \newpage
   \global\@topnum\z@
   \@maketitle
  \fi
  \thispagestyle{titlepage}
 \endgroup
 \setcounter{footnote}{0}
 \let\maketitle\relax
 \let\@maketitle\relax
 \gdef\@author{}
 \gdef\@title{}
 \hypersetup{pdfauthor={\@shortauthor},
     pdftitle={\@shorttitle}}% 
 \let\thanks\relax
}

%% fix problem with package tabularx
\def\and{%
  \end{tabular}\vskip 6pt\par
  \begin{tabular}[t]{@{}c@{}}\let\\=\author@nextline%
}
\def\@maketitle{\newpage
 \vspace*{10pt}
 {\centering \sloppy
  {\reset@font\huge \bfseries \@title \par}
  \vskip 35pt
  {\reset@font\normalsize
   \begin{tabular}[t]{@{}c@{}}\let\\=\author@nextline\@author
   \end{tabular}
   \par}
  \vskip 46pt
 }%

% added for teaser
\vspace{\titlespace}%
\ifx\@teaser\teaser@empty \else%
    \begingroup%
        \def\@captype{figure}%
        \@teaser%
    \endgroup\par%
    \vspace{\teaserspace}%
\fi%

}

\def\author@nextline{ %
 \tabularnewline[11pt]\reset@font\small\rmfamily\ignorespaces
 \let\\=\author@nextlinetwo
 }%
\def\author@nextlinetwo{\tabularnewline[-1pt]\reset@font\small\rmfamily\ignorespaces
 \let\\=\author@nextlinetwo
 } 

\newif\ifSFB@abstract
\def\abstract{\if@twocolumn
  \SFB@abstracttrue\start@SFBbox\SFB@widerule\addvspace{2pt}\bgroup\@abstract
 \else
  \@abstract
 \fi}
\def\endabstract{\if@twocolumn
  \SFB@abstractfalse\endlist\egroup
  \SFB@widerule\addvspace{25pt}\finish@SFBbox
  \@thanks \gdef\@thanks{}%
 \else
  \endlist
 \fi}
\def\@abstract{\list{}{\leftmargin 2pc\rightmargin\leftmargin
  \parsep 0pt plus .1pt}\item[]{\textbf{Abstract}}\\\reset@font\itshape}


\def\keywords{\vskip 5.5pt\par\reset@font\rmfamily\textbf{Keywords: }}
\def\endkeywords{\relax}
\newcommand\keyword[1]{[#1]}
\def\nokeywords{\relax}

\def\SFB@widerule{\bgroup\parskip 0pt\par\rule{\textwidth}{0.25pt}\par\egroup}

\def\author@tabular{\def\@halignto{}\@authortable}
\let\endauthor@tabular=\endtabular
\def\author@tabcrone{{\ifnum0=`}\fi\@xtabularcr[11pt]\reset@font\small\rmfamily
 \let\\=\author@tabcrtwo\ignorespaces}
\def\author@tabcrtwo{{\ifnum0=`}\fi\@xtabularcr[-1pt]\reset@font\small\rmfamily
 \let\\=\author@tabcrtwo\ignorespaces}
\def\@authortable{\leavevmode \hbox \bgroup $\let\@acol\@tabacol
 \let\@classz\@tabclassz \let\@classiv\@tabclassiv
 \let\\=\author@tabcrone \ignorespaces \@tabarray}

\def\start@SFBbox{\@next\@currbox\@freelist{}{}%
 \global\setbox\@currbox
 \vbox\bgroup
  \hsize \textwidth
  \@parboxrestore
}
\def\finish@SFBbox{\par\vskip -\dbltextfloatsep
  \egroup
  \global\count\@currbox\tw@
  \global\@dbltopnum\@ne
  \global\@topnum\z@
  \global\@dbltoproom\maxdimen\@addtodblcol
  \global\vsize\@colht
  \global\@colroom\@colht
}

%  ****************************************
%  *            PAGE STYLES               *
%  ****************************************
%
\mark{{}{}}
\gdef\@author{\mbox{}}
\def\author{\@ifnextchar [{\@authortwo}{\@authorone}}
\def\@authortwo[#1]#2{\gdef\@author{#2}\gdef\@shortauthor{#1}}
\def\@authorone#1{\gdef\@author{#1}\gdef\@shortauthor{#1}}
\gdef\@shortauthor{}
\gdef\@title{\mbox{}}
\def\title{\@ifnextchar [{\@titletwo}{\@titleone}}
\def\@titletwo[#1]#2{\gdef\@title{#2}\gdef\@shorttitle{#1}}
\def\@titleone#1{\gdef\@title{#1}\gdef\@shorttitle{#1}}
\gdef\@shorttitle{}

\newcommand\pStartPage[1]{\gdef\j@pStartPage{#1}}
\gdef\j@pStartPage{1}
%
\newcommand\pEndPage[1]{\gdef\j@pEndPage{#1}}
\gdef\j@pEndPage{000}

% ----
\newcommand\EmptySectionmark[1]{}
\newcommand\EmptySubsectionmark[1]{}
%
\def\thepagenumber{\def\p@thepagenumber{\thepage}}
\thepagenumber%

% -------
\def\Initialize{%
\thepagenumber%
\def\ps@titlepage{\let\@mkboth\@gobbletwo
 \def\@oddhead{\raisebox{\z@}[8pt][1pt]{\parbox{\textwidth}{\small
   \parbox[t]{.7\textwidth}{\sloppy\raggedright
     Technical Report \\
     May 2020}%
   \hfill
 }}}%
%
 \let\@evenhead=\@oddhead
 \let\@evenfoot=\@oddfoot
 \let\sectionmark=\EmptySectionmark
 \let\subsectionmark=\EmptySubsectionmark
}}

% -------
% Headings for Running Pages
%
\def\ps@headings{\let\@mkboth\markboth
 \def\@oddhead{\small \hfill \itshape \@shortauthor\ / \@shorttitle
               \hfill \upshape \textrm{\p@thepagenumber}}
 \def\@evenhead{\small \upshape \textrm{\p@thepagenumber} \hfill
                \itshape \@shortauthor\ / \@shorttitle \hfill}
 \def\sectionmark##1{}
 \def\subsectionmark##1{}}
%
\def\ps@myheadings{\let\@mkboth\@gobbletwo
 \def\@oddhead{\small \itshape \rightmark \hfill \mbox{}}
%%
 \def\@oddfoot{}
 \def\@evenhead{\small \itshape \leftmark \hfill \mbox{}}
%%
 \def\@evenfoot{}
 \def\sectionmark##1{}
 \def\subsectionmark##1{}}

%  ****************************************
%  *         TABLE OF CONTENTS, ETC.      *
%  ****************************************
%
\def\@pnumwidth{1.55em}
\def\@tocrmarg {2.55em}
\def\@dotsep{4.5}
\setcounter{tocdepth}{2}
\def\@undottedtocline#1#2#3#4#5{\ifnum #1>\c@tocdepth
 \else
  \vskip \z@ plus .2pt
  {\hangindent #2\relax
   \rightskip \@tocrmarg \parfillskip -\rightskip
   \parindent #2\relax \@afterindenttrue
   \interlinepenalty\@M \leavevmode
   \@tempdima #3\relax #4\nobreak \hfill \nobreak
   \hbox to\@pnumwidth{\hfil\reset@font\rmfamily \ }\par}\fi}
\def\tableofcontents{\@restonecolfalse
 \if@twocolumn\@restonecoltrue\onecolumn\fi
 \section*{Contents} \@starttoc{toc}
 \if@restonecol\twocolumn\fi \par\vspace{11pt}}
\def\l@part#1#2{\addpenalty{-\@highpenalty}
 \addvspace{2.25em plus 1pt}
 \begingroup
  \parindent \z@ \rightskip \@pnumwidth
  \parfillskip -\@pnumwidth
  {\reset@font\normalsize\rmfamily
   \leavevmode \hspace*{3pc}
   #1\hfil \hbox to\@pnumwidth{\hss \ }}\par
   \nobreak \global\@nobreaktrue
   \everypar{\global\@nobreakfalse\everypar{}}\endgroup}
\def\l@section#1#2{\addpenalty{\@secpenalty}
 \@tempdima 1.5em
 \begingroup
  \parindent \z@ \rightskip \@pnumwidth
  \parfillskip -\@pnumwidth \reset@font\rmfamily \leavevmode
  \advance\leftskip\@tempdima \hskip -\leftskip
  #1\nobreak\hfil \nobreak\hbox to\@pnumwidth{\hss \ }\par
 \endgroup}
\def\l@subsection{\@undottedtocline{2}{1.5em}{2.3em}}
\def\l@subsubsection{\@undottedtocline{3}{3.8em}{3.2em}}
\def\l@paragraph{\@undottedtocline{4}{7.0em}{4.1em}}
\def\l@subparagraph{\@undottedtocline{5}{10em}{5em}}
\def\listoffigures{\@restonecolfalse
 \if@twocolumn\@restonecoltrue\onecolumn\fi
 \section*{List of Figures\@mkboth{List of Figures}{List of Figures}}
 \@starttoc{lof} \if@restonecol\twocolumn\fi}
\def\l@figure{\@undottedtocline{1}{1.5em}{2.3em}}
\def\listoftables{\@restonecolfalse
 \if@twocolumn\@restonecoltrue\onecolumn\fi
 \section*{List of Tables\@mkboth{List of Tables}{List of Tables}}
 \@starttoc{lot} \if@restonecol\twocolumn\fi}
\let\l@table\l@figure

%  ****************************************
%  *             BIBLIOGRAPHY             *
%  ****************************************

\def\thebibliography#1{\subsection*{References}
 \addcontentsline{toc}{section}{References}
 \list{\relax}{%
    \setlength{\itemindent}{0pt}
    \setlength{\leftmargin}{1em}
    \itemsep 4pt plus 1pt minus 1pt%
     \fontsize{8} {9} \selectfont
}%
 \parindent\z@
 \parskip\z@ plus .1pt\relax
 \def\newblock{\hskip .11em plus .33em minus .07em}%
 \sloppy\clubpenalty4000\widowpenalty4000%
 \sfcode`\.=1000\relax
}
%
\let\endthebibliography=\endlist
\def\@biblabel#1{[#1] \ }
%
\def\@cite#1{\mbox{[#1]}}

%  ****************************************
%  *              THE INDEX               *
%  ****************************************
%
\newif\if@restonecol
\def\theindex{\section*{Index}
 \addcontentsline{toc}{section}{Index}
 \reset@font\footnotesize \parindent\z@ \parskip\z@ plus .1pt\relax
 \let\item\@idxitem}
\def\@idxitem{\par\hangindent 1em}
\def\subitem {\par\hangindent 1em \hspace*{1em}}
\def\subsubitem{\par\hangindent 1em\hspace*{1em}}
\def\endtheindex{\if@restonecol\onecolumn\else\clearpage\fi}
\def\indexspace{\par\vskip\baselineskip \relax}

%  ****************************************
%  *            MISCELLANEOUS             *
%  ****************************************
%
% DATE
\def\today{\number\day\ \ifcase\month\or
  January\or February\or March\or April\or May\or June\or
  July\or August\or September\or October\or November\or December
 \fi \ \number\year}

%  ****************************************
%  *         HYPERLINK SUPPORT            *
%  ****************************************
%

\RequirePackage{ifpdf}

\newcommand{\bibtex@Version}{%
  \ifpdf
  \newcommand{\electronic@Version}{%
   \usepackage[pdftex,
    pagebackref=false, 
    colorlinks,linkcolor=blue,citecolor=blue,urlcolor=blue,
    bookmarks=false,
    pdfpagemode=UseNone,
    pdftitle={\@shorttitle},
    pdfauthor={\@shortauthor}]{hyperref}
  }
\else
  \newcommand{\electronic@Version}{%
   \usepackage[dvips,
    pagebackref=false,
    colorlinks,linkcolor=blue,citecolor=blue,urlcolor=blue,
    bookmarks=false,
    pdfpagemode=UseNone,
    pdftitle={\@shorttitle},
    pdfauthor={\@shortauthor}]{hyperref}
  }
\fi
}

\bibtex@Version
\electronic@Version

%  ****************************************
%  *           INITIALIZATION             *
%  ****************************************
%
\pagenumbering{arabic}
\if@twocolumn
 \twocolumn
 \sloppy
\fi
\ifSFB@galley
 \ps@empty
 \raggedbottom
\else
 \ps@headings
 \flushbottom
\fi
\frenchspacing

\AtBeginDocument{{\setcounter{page}{\j@pStartPage}}}

\gdef\jobname@lbl{\jobname.lbl}%
\InputIfFileExists{\jobname.lbl}{}{}%
\def\label@out{9}%
\immediate\openout\label@out=\jobname@lbl%

\AtEndDocument{%
\immediate\write\label@out{\string\pEndPage{\thepage}\string\endinput}%
}

\Initialize

\endinput
