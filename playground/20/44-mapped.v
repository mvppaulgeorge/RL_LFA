// Benchmark "adder" written by ABC on Wed Jul 17 22:37:07 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n302,
    new_n304, new_n307, new_n309, new_n311;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand02aa1d24x5               g001(.a(\b[0] ), .b(\a[1] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nor042aa1n12x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  tech160nm_finand02aa1n03p5x5 g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  oai112aa1n04x5               g005(.a(new_n100), .b(new_n98), .c(new_n99), .d(new_n97), .o1(new_n101));
  oa0022aa1n06x5               g006(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n102));
  aoi022aa1n09x5               g007(.a(new_n101), .b(new_n102), .c(\b[3] ), .d(\a[4] ), .o1(new_n103));
  nor002aa1d32x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nand02aa1d06x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  nand02aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  nona23aa1n03x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  nor002aa1n03x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nand02aa1n10x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand42aa1n08x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nona23aa1n02x4               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  nor042aa1n03x5               g018(.a(new_n113), .b(new_n108), .o1(new_n114));
  nanb03aa1n12x5               g019(.a(new_n111), .b(new_n112), .c(new_n110), .out0(new_n115));
  oaih12aa1n06x5               g020(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n116));
  inv040aa1n06x5               g021(.a(new_n111), .o1(new_n117));
  oaoi03aa1n09x5               g022(.a(\a[8] ), .b(\b[7] ), .c(new_n117), .o1(new_n118));
  oabi12aa1n18x5               g023(.a(new_n118), .b(new_n115), .c(new_n116), .out0(new_n119));
  nor042aa1n03x5               g024(.a(\b[8] ), .b(\a[9] ), .o1(new_n120));
  nand22aa1n03x5               g025(.a(\b[8] ), .b(\a[9] ), .o1(new_n121));
  norb02aa1n09x5               g026(.a(new_n121), .b(new_n120), .out0(new_n122));
  aoai13aa1n06x5               g027(.a(new_n122), .b(new_n119), .c(new_n103), .d(new_n114), .o1(new_n123));
  oai012aa1n02x5               g028(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .o1(new_n124));
  xorb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1d28x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nor002aa1d32x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  nand02aa1d28x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  nor002aa1n03x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nona22aa1n02x4               g035(.a(new_n123), .b(new_n120), .c(new_n130), .out0(new_n131));
  xobna2aa1n03x5               g036(.a(new_n129), .b(new_n131), .c(new_n126), .out0(\s[11] ));
  inv000aa1d42x5               g037(.a(\b[11] ), .o1(new_n133));
  nanb02aa1n02x5               g038(.a(\a[12] ), .b(new_n133), .out0(new_n134));
  nand42aa1d28x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  aoi013aa1n03x5               g040(.a(new_n127), .b(new_n131), .c(new_n129), .d(new_n126), .o1(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n134), .c(new_n135), .out0(\s[12] ));
  nanp02aa1n03x5               g042(.a(new_n103), .b(new_n114), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n119), .o1(new_n139));
  norb03aa1n03x5               g044(.a(new_n126), .b(new_n130), .c(new_n127), .out0(new_n140));
  nor002aa1d32x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nano22aa1n02x4               g046(.a(new_n141), .b(new_n128), .c(new_n135), .out0(new_n142));
  nand23aa1n03x5               g047(.a(new_n142), .b(new_n140), .c(new_n122), .o1(new_n143));
  nanb03aa1d24x5               g048(.a(new_n141), .b(new_n135), .c(new_n128), .out0(new_n144));
  inv040aa1d30x5               g049(.a(\a[11] ), .o1(new_n145));
  inv040aa1n18x5               g050(.a(\b[10] ), .o1(new_n146));
  nand02aa1n06x5               g051(.a(new_n146), .b(new_n145), .o1(new_n147));
  oai022aa1d24x5               g052(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n148));
  nand23aa1n09x5               g053(.a(new_n148), .b(new_n147), .c(new_n126), .o1(new_n149));
  aoi012aa1n12x5               g054(.a(new_n141), .b(new_n127), .c(new_n135), .o1(new_n150));
  oai012aa1d24x5               g055(.a(new_n150), .b(new_n149), .c(new_n144), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n143), .c(new_n138), .d(new_n139), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n04x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1d28x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n03x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand42aa1n16x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nano23aa1n06x5               g065(.a(new_n155), .b(new_n159), .c(new_n160), .d(new_n156), .out0(new_n161));
  oa0012aa1n06x5               g066(.a(new_n160), .b(new_n159), .c(new_n155), .o(new_n162));
  nor002aa1n20x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nand42aa1n16x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  aoai13aa1n03x5               g070(.a(new_n165), .b(new_n162), .c(new_n153), .d(new_n161), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n165), .b(new_n162), .c(new_n153), .d(new_n161), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(\s[15] ));
  inv000aa1d42x5               g073(.a(new_n163), .o1(new_n169));
  nor042aa1n04x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1d28x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n166), .c(new_n169), .out0(\s[16] ));
  nano23aa1d15x5               g078(.a(new_n163), .b(new_n170), .c(new_n171), .d(new_n164), .out0(new_n174));
  nano22aa1n06x5               g079(.a(new_n143), .b(new_n161), .c(new_n174), .out0(new_n175));
  aoai13aa1n12x5               g080(.a(new_n175), .b(new_n119), .c(new_n103), .d(new_n114), .o1(new_n176));
  aoai13aa1n12x5               g081(.a(new_n174), .b(new_n162), .c(new_n151), .d(new_n161), .o1(new_n177));
  aoi012aa1n12x5               g082(.a(new_n170), .b(new_n163), .c(new_n171), .o1(new_n178));
  nand23aa1d12x5               g083(.a(new_n176), .b(new_n177), .c(new_n178), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g085(.a(\a[18] ), .o1(new_n181));
  inv040aa1d32x5               g086(.a(\a[17] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[16] ), .o1(new_n183));
  oaoi03aa1n03x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  xroi22aa1d06x4               g090(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n183), .b(new_n182), .o1(new_n187));
  oaoi03aa1n12x5               g092(.a(\a[18] ), .b(\b[17] ), .c(new_n187), .o1(new_n188));
  nor002aa1d32x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(new_n189), .b(new_n190), .out0(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n188), .c(new_n179), .d(new_n186), .o1(new_n193));
  aoi112aa1n02x5               g098(.a(new_n192), .b(new_n188), .c(new_n179), .d(new_n186), .o1(new_n194));
  norb02aa1n02x7               g099(.a(new_n193), .b(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g101(.a(new_n189), .o1(new_n197));
  nor002aa1n12x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nand22aa1n03x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  nand43aa1n02x5               g105(.a(new_n193), .b(new_n197), .c(new_n200), .o1(new_n201));
  tech160nm_fiaoi012aa1n02p5x5 g106(.a(new_n200), .b(new_n193), .c(new_n197), .o1(new_n202));
  norb02aa1n03x4               g107(.a(new_n201), .b(new_n202), .out0(\s[20] ));
  nona23aa1n03x5               g108(.a(new_n199), .b(new_n190), .c(new_n189), .d(new_n198), .out0(new_n204));
  norb02aa1n02x5               g109(.a(new_n186), .b(new_n204), .out0(new_n205));
  oai022aa1n02x5               g110(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n206));
  oaib12aa1n02x5               g111(.a(new_n206), .b(new_n181), .c(\b[17] ), .out0(new_n207));
  aoi012aa1n02x5               g112(.a(new_n198), .b(new_n189), .c(new_n199), .o1(new_n208));
  oai012aa1n06x5               g113(.a(new_n208), .b(new_n204), .c(new_n207), .o1(new_n209));
  xnrc02aa1n12x5               g114(.a(\b[20] ), .b(\a[21] ), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n209), .c(new_n179), .d(new_n205), .o1(new_n212));
  aoi112aa1n02x5               g117(.a(new_n211), .b(new_n209), .c(new_n179), .d(new_n205), .o1(new_n213));
  norb02aa1n02x7               g118(.a(new_n212), .b(new_n213), .out0(\s[21] ));
  tech160nm_fixnrc02aa1n04x5   g119(.a(\b[21] ), .b(\a[22] ), .out0(new_n215));
  oai112aa1n02x7               g120(.a(new_n212), .b(new_n215), .c(\b[20] ), .d(\a[21] ), .o1(new_n216));
  oaoi13aa1n02x7               g121(.a(new_n215), .b(new_n212), .c(\a[21] ), .d(\b[20] ), .o1(new_n217));
  norb02aa1n03x4               g122(.a(new_n216), .b(new_n217), .out0(\s[22] ));
  nano23aa1n06x5               g123(.a(new_n189), .b(new_n198), .c(new_n199), .d(new_n190), .out0(new_n219));
  nor042aa1n06x5               g124(.a(new_n215), .b(new_n210), .o1(new_n220));
  and003aa1n06x5               g125(.a(new_n186), .b(new_n220), .c(new_n219), .o(new_n221));
  inv020aa1n02x5               g126(.a(new_n208), .o1(new_n222));
  aoai13aa1n06x5               g127(.a(new_n220), .b(new_n222), .c(new_n219), .d(new_n188), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\a[22] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[21] ), .o1(new_n225));
  norp02aa1n02x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  oaoi03aa1n09x5               g131(.a(new_n224), .b(new_n225), .c(new_n226), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(new_n223), .b(new_n227), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[22] ), .b(\a[23] ), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n228), .c(new_n179), .d(new_n221), .o1(new_n231));
  aoi112aa1n02x5               g136(.a(new_n230), .b(new_n228), .c(new_n179), .d(new_n221), .o1(new_n232));
  norb02aa1n02x7               g137(.a(new_n231), .b(new_n232), .out0(\s[23] ));
  xnrc02aa1n12x5               g138(.a(\b[23] ), .b(\a[24] ), .out0(new_n234));
  oai112aa1n02x7               g139(.a(new_n231), .b(new_n234), .c(\b[22] ), .d(\a[23] ), .o1(new_n235));
  oaoi13aa1n02x7               g140(.a(new_n234), .b(new_n231), .c(\a[23] ), .d(\b[22] ), .o1(new_n236));
  norb02aa1n03x4               g141(.a(new_n235), .b(new_n236), .out0(\s[24] ));
  nor042aa1n04x5               g142(.a(new_n234), .b(new_n229), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n238), .o1(new_n239));
  nano32aa1n02x4               g144(.a(new_n239), .b(new_n186), .c(new_n220), .d(new_n219), .out0(new_n240));
  norp02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n242));
  norp02aa1n02x5               g147(.a(new_n242), .b(new_n241), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n239), .c(new_n223), .d(new_n227), .o1(new_n244));
  tech160nm_fiaoi012aa1n05x5   g149(.a(new_n244), .b(new_n179), .c(new_n240), .o1(new_n245));
  xnrc02aa1n12x5               g150(.a(\b[24] ), .b(\a[25] ), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  xnrc02aa1n02x5               g152(.a(new_n245), .b(new_n247), .out0(\s[25] ));
  nor042aa1n03x5               g153(.a(\b[24] ), .b(\a[25] ), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  aoai13aa1n03x5               g155(.a(new_n247), .b(new_n244), .c(new_n179), .d(new_n240), .o1(new_n251));
  xnrc02aa1n12x5               g156(.a(\b[25] ), .b(\a[26] ), .out0(new_n252));
  nanp03aa1n03x5               g157(.a(new_n251), .b(new_n250), .c(new_n252), .o1(new_n253));
  tech160nm_fiaoi012aa1n02p5x5 g158(.a(new_n252), .b(new_n251), .c(new_n250), .o1(new_n254));
  norb02aa1n03x4               g159(.a(new_n253), .b(new_n254), .out0(\s[26] ));
  nor042aa1n03x5               g160(.a(new_n252), .b(new_n246), .o1(new_n256));
  inv040aa1n03x5               g161(.a(new_n256), .o1(new_n257));
  nona32aa1n03x5               g162(.a(new_n221), .b(new_n257), .c(new_n234), .d(new_n229), .out0(new_n258));
  inv040aa1n03x5               g163(.a(new_n258), .o1(new_n259));
  nand22aa1n09x5               g164(.a(new_n179), .b(new_n259), .o1(new_n260));
  oao003aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .c(new_n250), .carry(new_n261));
  aobi12aa1n12x5               g166(.a(new_n261), .b(new_n244), .c(new_n256), .out0(new_n262));
  xorc02aa1n12x5               g167(.a(\a[27] ), .b(\b[26] ), .out0(new_n263));
  xnbna2aa1n06x5               g168(.a(new_n263), .b(new_n262), .c(new_n260), .out0(\s[27] ));
  norp02aa1n02x5               g169(.a(\b[26] ), .b(\a[27] ), .o1(new_n265));
  inv040aa1n03x5               g170(.a(new_n265), .o1(new_n266));
  aobi12aa1n06x5               g171(.a(new_n263), .b(new_n262), .c(new_n260), .out0(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[27] ), .b(\a[28] ), .out0(new_n268));
  nano22aa1n03x5               g173(.a(new_n267), .b(new_n266), .c(new_n268), .out0(new_n269));
  aoi013aa1n06x4               g174(.a(new_n258), .b(new_n176), .c(new_n177), .d(new_n178), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n227), .o1(new_n271));
  aoai13aa1n02x7               g176(.a(new_n238), .b(new_n271), .c(new_n209), .d(new_n220), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n261), .b(new_n257), .c(new_n272), .d(new_n243), .o1(new_n273));
  oaih12aa1n02x5               g178(.a(new_n263), .b(new_n273), .c(new_n270), .o1(new_n274));
  tech160nm_fiaoi012aa1n02p5x5 g179(.a(new_n268), .b(new_n274), .c(new_n266), .o1(new_n275));
  norp02aa1n03x5               g180(.a(new_n275), .b(new_n269), .o1(\s[28] ));
  norb02aa1n02x5               g181(.a(new_n263), .b(new_n268), .out0(new_n277));
  oaih12aa1n02x5               g182(.a(new_n277), .b(new_n273), .c(new_n270), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .c(new_n266), .carry(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[28] ), .b(\a[29] ), .out0(new_n280));
  tech160nm_fiaoi012aa1n02p5x5 g185(.a(new_n280), .b(new_n278), .c(new_n279), .o1(new_n281));
  aobi12aa1n06x5               g186(.a(new_n277), .b(new_n262), .c(new_n260), .out0(new_n282));
  nano22aa1n03x5               g187(.a(new_n282), .b(new_n279), .c(new_n280), .out0(new_n283));
  norp02aa1n03x5               g188(.a(new_n281), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n97), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g190(.a(new_n263), .b(new_n280), .c(new_n268), .out0(new_n286));
  oaih12aa1n02x5               g191(.a(new_n286), .b(new_n273), .c(new_n270), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[29] ), .b(\a[30] ), .out0(new_n289));
  tech160nm_fiaoi012aa1n02p5x5 g194(.a(new_n289), .b(new_n287), .c(new_n288), .o1(new_n290));
  aobi12aa1n06x5               g195(.a(new_n286), .b(new_n262), .c(new_n260), .out0(new_n291));
  nano22aa1n03x5               g196(.a(new_n291), .b(new_n288), .c(new_n289), .out0(new_n292));
  norp02aa1n03x5               g197(.a(new_n290), .b(new_n292), .o1(\s[30] ));
  norb02aa1n02x5               g198(.a(new_n286), .b(new_n289), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n273), .c(new_n270), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[30] ), .b(\b[29] ), .c(new_n288), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[30] ), .b(\a[31] ), .out0(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n06x5               g203(.a(new_n294), .b(new_n262), .c(new_n260), .out0(new_n299));
  nano22aa1n03x5               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[31] ));
  oai012aa1n02x5               g206(.a(new_n98), .b(new_n99), .c(new_n97), .o1(new_n302));
  xnrb03aa1n02x5               g207(.a(new_n302), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g208(.a(\a[3] ), .b(\b[2] ), .c(new_n302), .o1(new_n304));
  xorb03aa1n02x5               g209(.a(new_n304), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g210(.a(new_n103), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g211(.a(new_n107), .b(new_n103), .c(new_n106), .o1(new_n307));
  xnrb03aa1n02x5               g212(.a(new_n307), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n06x5               g213(.a(new_n105), .b(new_n104), .c(new_n307), .out0(new_n309));
  xnbna2aa1n03x5               g214(.a(new_n309), .b(new_n117), .c(new_n112), .out0(\s[7] ));
  oaoi03aa1n02x5               g215(.a(\a[7] ), .b(\b[6] ), .c(new_n309), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g217(.a(new_n122), .b(new_n138), .c(new_n139), .out0(\s[9] ));
endmodule

