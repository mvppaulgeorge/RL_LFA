// Benchmark "adder" written by ABC on Thu Jul 18 12:02:30 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n312, new_n313, new_n315, new_n316, new_n317, new_n320,
    new_n321, new_n323, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  inv000aa1d42x5               g002(.a(\a[3] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[4] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[3] ), .o1(new_n100));
  aboi22aa1n06x5               g005(.a(\b[2] ), .b(new_n98), .c(new_n100), .d(new_n99), .out0(new_n101));
  tech160nm_fixnrc02aa1n05x5   g006(.a(\b[2] ), .b(\a[3] ), .out0(new_n102));
  nand42aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand22aa1n04x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  nor042aa1n03x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  oaih12aa1n06x5               g010(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n106));
  oai012aa1n18x5               g011(.a(new_n101), .b(new_n102), .c(new_n106), .o1(new_n107));
  nand42aa1d28x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  norp02aa1n04x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  norb02aa1n06x5               g014(.a(new_n108), .b(new_n109), .out0(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .out0(new_n111));
  tech160nm_finand02aa1n03p5x5 g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand02aa1n08x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  orn002aa1n12x5               g018(.a(\a[5] ), .b(\b[4] ), .o(new_n114));
  nand23aa1n04x5               g019(.a(new_n114), .b(new_n112), .c(new_n113), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  aoi012aa1n02x7               g021(.a(new_n116), .b(\a[4] ), .c(\b[3] ), .o1(new_n117));
  nano23aa1n09x5               g022(.a(new_n115), .b(new_n111), .c(new_n110), .d(new_n117), .out0(new_n118));
  and002aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o(new_n119));
  tech160nm_finor002aa1n05x5   g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  oai022aa1d18x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  aoai13aa1n06x5               g026(.a(new_n112), .b(new_n120), .c(new_n121), .d(new_n108), .o1(new_n122));
  oai022aa1d18x5               g027(.a(new_n122), .b(new_n119), .c(\b[7] ), .d(\a[8] ), .o1(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n123), .c(new_n107), .d(new_n118), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n125), .c(new_n97), .out0(\s[10] ));
  nanp02aa1n06x5               g032(.a(new_n125), .b(new_n97), .o1(new_n128));
  oaoi03aa1n09x5               g033(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n129));
  nor042aa1n04x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nand02aa1d06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  aoai13aa1n06x5               g037(.a(new_n132), .b(new_n129), .c(new_n128), .d(new_n126), .o1(new_n133));
  aoi112aa1n02x5               g038(.a(new_n132), .b(new_n129), .c(new_n128), .d(new_n126), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(\s[11] ));
  nor002aa1n06x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand22aa1n06x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  nona22aa1n02x5               g043(.a(new_n133), .b(new_n138), .c(new_n130), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n138), .o1(new_n140));
  oaoi13aa1n06x5               g045(.a(new_n140), .b(new_n133), .c(\a[11] ), .d(\b[10] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n139), .b(new_n141), .out0(\s[12] ));
  aoi012aa1d18x5               g047(.a(new_n123), .b(new_n107), .c(new_n118), .o1(new_n143));
  nano23aa1n06x5               g048(.a(new_n130), .b(new_n136), .c(new_n137), .d(new_n131), .out0(new_n144));
  nand23aa1n03x5               g049(.a(new_n144), .b(new_n124), .c(new_n126), .o1(new_n145));
  oai022aa1n02x5               g050(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n146));
  aob012aa1n02x5               g051(.a(new_n146), .b(\b[9] ), .c(\a[10] ), .out0(new_n147));
  nona23aa1n02x5               g052(.a(new_n137), .b(new_n131), .c(new_n130), .d(new_n136), .out0(new_n148));
  ao0012aa1n03x7               g053(.a(new_n136), .b(new_n130), .c(new_n137), .o(new_n149));
  oabi12aa1n02x5               g054(.a(new_n149), .b(new_n148), .c(new_n147), .out0(new_n150));
  oabi12aa1n09x5               g055(.a(new_n150), .b(new_n143), .c(new_n145), .out0(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g057(.a(\a[14] ), .o1(new_n153));
  nor002aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  xnrc02aa1n12x5               g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  aoib12aa1n02x5               g060(.a(new_n154), .b(new_n151), .c(new_n155), .out0(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(new_n153), .out0(\s[14] ));
  xnrc02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .out0(new_n158));
  nor042aa1n04x5               g063(.a(new_n158), .b(new_n155), .o1(new_n159));
  inv000aa1d42x5               g064(.a(\b[13] ), .o1(new_n160));
  oao003aa1n12x5               g065(.a(new_n153), .b(new_n160), .c(new_n154), .carry(new_n161));
  tech160nm_fixorc02aa1n04x5   g066(.a(\a[15] ), .b(\b[14] ), .out0(new_n162));
  aoai13aa1n06x5               g067(.a(new_n162), .b(new_n161), .c(new_n151), .d(new_n159), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n162), .b(new_n161), .c(new_n151), .d(new_n159), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(\s[15] ));
  xorc02aa1n12x5               g070(.a(\a[16] ), .b(\b[15] ), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  oai112aa1n02x7               g072(.a(new_n163), .b(new_n167), .c(\b[14] ), .d(\a[15] ), .o1(new_n168));
  oaoi13aa1n02x7               g073(.a(new_n167), .b(new_n163), .c(\a[15] ), .d(\b[14] ), .o1(new_n169));
  norb02aa1n03x4               g074(.a(new_n168), .b(new_n169), .out0(\s[16] ));
  and002aa1n06x5               g075(.a(new_n166), .b(new_n162), .o(new_n171));
  nanb03aa1n12x5               g076(.a(new_n145), .b(new_n171), .c(new_n159), .out0(new_n172));
  aoai13aa1n02x7               g077(.a(new_n171), .b(new_n161), .c(new_n150), .d(new_n159), .o1(new_n173));
  aoi112aa1n02x5               g078(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n174));
  oab012aa1n02x4               g079(.a(new_n174), .b(\a[16] ), .c(\b[15] ), .out0(new_n175));
  oai112aa1n06x5               g080(.a(new_n173), .b(new_n175), .c(new_n143), .d(new_n172), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g082(.a(\a[18] ), .o1(new_n178));
  inv040aa1d32x5               g083(.a(\a[17] ), .o1(new_n179));
  inv030aa1d32x5               g084(.a(\b[16] ), .o1(new_n180));
  oaoi03aa1n03x5               g085(.a(new_n179), .b(new_n180), .c(new_n176), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(new_n178), .out0(\s[18] ));
  nand22aa1n04x5               g087(.a(new_n118), .b(new_n107), .o1(new_n183));
  inv030aa1n02x5               g088(.a(new_n123), .o1(new_n184));
  aoi012aa1n12x5               g089(.a(new_n172), .b(new_n184), .c(new_n183), .o1(new_n185));
  inv000aa1n02x5               g090(.a(new_n161), .o1(new_n186));
  inv030aa1n02x5               g091(.a(new_n171), .o1(new_n187));
  aoai13aa1n03x5               g092(.a(new_n159), .b(new_n149), .c(new_n144), .d(new_n129), .o1(new_n188));
  aoai13aa1n06x5               g093(.a(new_n175), .b(new_n187), .c(new_n188), .d(new_n186), .o1(new_n189));
  xroi22aa1d06x4               g094(.a(new_n179), .b(\b[16] ), .c(new_n178), .d(\b[17] ), .out0(new_n190));
  tech160nm_fioai012aa1n05x5   g095(.a(new_n190), .b(new_n185), .c(new_n189), .o1(new_n191));
  oaih22aa1n06x5               g096(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n192));
  oaib12aa1n09x5               g097(.a(new_n192), .b(new_n178), .c(\b[17] ), .out0(new_n193));
  nor002aa1d32x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nand22aa1n12x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(new_n194), .b(new_n195), .out0(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n191), .c(new_n193), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g104(.a(new_n194), .o1(new_n200));
  aoi012aa1n02x5               g105(.a(new_n196), .b(new_n191), .c(new_n193), .o1(new_n201));
  nor002aa1n20x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nand22aa1n12x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  nano22aa1n03x5               g109(.a(new_n201), .b(new_n200), .c(new_n204), .out0(new_n205));
  nanp02aa1n02x5               g110(.a(new_n180), .b(new_n179), .o1(new_n206));
  oaoi03aa1n09x5               g111(.a(\a[18] ), .b(\b[17] ), .c(new_n206), .o1(new_n207));
  aoai13aa1n03x5               g112(.a(new_n197), .b(new_n207), .c(new_n176), .d(new_n190), .o1(new_n208));
  aoi012aa1n03x5               g113(.a(new_n204), .b(new_n208), .c(new_n200), .o1(new_n209));
  nor002aa1n02x5               g114(.a(new_n209), .b(new_n205), .o1(\s[20] ));
  nano23aa1n06x5               g115(.a(new_n194), .b(new_n202), .c(new_n203), .d(new_n195), .out0(new_n211));
  nand22aa1n03x5               g116(.a(new_n190), .b(new_n211), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  tech160nm_fioai012aa1n05x5   g118(.a(new_n213), .b(new_n185), .c(new_n189), .o1(new_n214));
  nona23aa1n09x5               g119(.a(new_n203), .b(new_n195), .c(new_n194), .d(new_n202), .out0(new_n215));
  tech160nm_fiaoi012aa1n05x5   g120(.a(new_n202), .b(new_n194), .c(new_n203), .o1(new_n216));
  oai012aa1n12x5               g121(.a(new_n216), .b(new_n215), .c(new_n193), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  nor002aa1d32x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  xnbna2aa1n03x5               g126(.a(new_n221), .b(new_n214), .c(new_n218), .out0(\s[21] ));
  inv000aa1d42x5               g127(.a(new_n219), .o1(new_n223));
  aobi12aa1n06x5               g128(.a(new_n221), .b(new_n214), .c(new_n218), .out0(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  nano22aa1n02x4               g130(.a(new_n224), .b(new_n223), .c(new_n225), .out0(new_n226));
  aoai13aa1n03x5               g131(.a(new_n221), .b(new_n217), .c(new_n176), .d(new_n213), .o1(new_n227));
  aoi012aa1n03x5               g132(.a(new_n225), .b(new_n227), .c(new_n223), .o1(new_n228));
  nor002aa1n02x5               g133(.a(new_n228), .b(new_n226), .o1(\s[22] ));
  nano22aa1n03x7               g134(.a(new_n225), .b(new_n223), .c(new_n220), .out0(new_n230));
  and003aa1n02x5               g135(.a(new_n190), .b(new_n230), .c(new_n211), .o(new_n231));
  oai012aa1n03x5               g136(.a(new_n231), .b(new_n185), .c(new_n189), .o1(new_n232));
  oao003aa1n12x5               g137(.a(\a[22] ), .b(\b[21] ), .c(new_n223), .carry(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n217), .c(new_n230), .o1(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[22] ), .b(\a[23] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  xnbna2aa1n03x5               g142(.a(new_n237), .b(new_n232), .c(new_n235), .out0(\s[23] ));
  nor042aa1n06x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoi012aa1n03x5               g145(.a(new_n236), .b(new_n232), .c(new_n235), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[23] ), .b(\a[24] ), .out0(new_n242));
  nano22aa1n02x4               g147(.a(new_n241), .b(new_n240), .c(new_n242), .out0(new_n243));
  inv000aa1n03x5               g148(.a(new_n235), .o1(new_n244));
  aoai13aa1n03x5               g149(.a(new_n237), .b(new_n244), .c(new_n176), .d(new_n231), .o1(new_n245));
  aoi012aa1n03x5               g150(.a(new_n242), .b(new_n245), .c(new_n240), .o1(new_n246));
  nor002aa1n02x5               g151(.a(new_n246), .b(new_n243), .o1(\s[24] ));
  nor002aa1n03x5               g152(.a(new_n242), .b(new_n236), .o1(new_n248));
  nano22aa1n03x7               g153(.a(new_n212), .b(new_n230), .c(new_n248), .out0(new_n249));
  oai012aa1n03x5               g154(.a(new_n249), .b(new_n185), .c(new_n189), .o1(new_n250));
  inv000aa1n02x5               g155(.a(new_n216), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n230), .b(new_n251), .c(new_n211), .d(new_n207), .o1(new_n252));
  inv030aa1n02x5               g157(.a(new_n248), .o1(new_n253));
  oao003aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n240), .carry(new_n254));
  aoai13aa1n12x5               g159(.a(new_n254), .b(new_n253), .c(new_n252), .d(new_n233), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  xnrc02aa1n12x5               g161(.a(\b[24] ), .b(\a[25] ), .out0(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  xnbna2aa1n03x5               g163(.a(new_n258), .b(new_n250), .c(new_n256), .out0(\s[25] ));
  nor042aa1n03x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  aoi012aa1n02x5               g166(.a(new_n257), .b(new_n250), .c(new_n256), .o1(new_n262));
  xnrc02aa1n02x5               g167(.a(\b[25] ), .b(\a[26] ), .out0(new_n263));
  nano22aa1n02x4               g168(.a(new_n262), .b(new_n261), .c(new_n263), .out0(new_n264));
  aoai13aa1n03x5               g169(.a(new_n258), .b(new_n255), .c(new_n176), .d(new_n249), .o1(new_n265));
  aoi012aa1n03x5               g170(.a(new_n263), .b(new_n265), .c(new_n261), .o1(new_n266));
  nor002aa1n02x5               g171(.a(new_n266), .b(new_n264), .o1(\s[26] ));
  nor042aa1n04x5               g172(.a(new_n263), .b(new_n257), .o1(new_n268));
  nano32aa1n03x7               g173(.a(new_n212), .b(new_n268), .c(new_n230), .d(new_n248), .out0(new_n269));
  oai012aa1n09x5               g174(.a(new_n269), .b(new_n185), .c(new_n189), .o1(new_n270));
  oao003aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .c(new_n261), .carry(new_n271));
  aobi12aa1n12x5               g176(.a(new_n271), .b(new_n255), .c(new_n268), .out0(new_n272));
  xorc02aa1n12x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n270), .c(new_n272), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  inv040aa1n03x5               g180(.a(new_n275), .o1(new_n276));
  aobi12aa1n03x5               g181(.a(new_n273), .b(new_n270), .c(new_n272), .out0(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[27] ), .b(\a[28] ), .out0(new_n278));
  nano22aa1n03x5               g183(.a(new_n277), .b(new_n276), .c(new_n278), .out0(new_n279));
  aoai13aa1n03x5               g184(.a(new_n248), .b(new_n234), .c(new_n217), .d(new_n230), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n268), .o1(new_n281));
  aoai13aa1n04x5               g186(.a(new_n271), .b(new_n281), .c(new_n280), .d(new_n254), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n273), .b(new_n282), .c(new_n176), .d(new_n269), .o1(new_n283));
  aoi012aa1n03x5               g188(.a(new_n278), .b(new_n283), .c(new_n276), .o1(new_n284));
  norp02aa1n03x5               g189(.a(new_n284), .b(new_n279), .o1(\s[28] ));
  norb02aa1n02x5               g190(.a(new_n273), .b(new_n278), .out0(new_n286));
  aoai13aa1n02x5               g191(.a(new_n286), .b(new_n282), .c(new_n176), .d(new_n269), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .c(new_n276), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[28] ), .b(\a[29] ), .out0(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(new_n287), .c(new_n288), .o1(new_n290));
  aobi12aa1n03x5               g195(.a(new_n286), .b(new_n270), .c(new_n272), .out0(new_n291));
  nano22aa1n03x5               g196(.a(new_n291), .b(new_n288), .c(new_n289), .out0(new_n292));
  norp02aa1n03x5               g197(.a(new_n290), .b(new_n292), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g199(.a(new_n273), .b(new_n289), .c(new_n278), .out0(new_n295));
  aoai13aa1n02x5               g200(.a(new_n295), .b(new_n282), .c(new_n176), .d(new_n269), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n288), .carry(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[29] ), .b(\a[30] ), .out0(new_n298));
  aoi012aa1n02x7               g203(.a(new_n298), .b(new_n296), .c(new_n297), .o1(new_n299));
  aobi12aa1n03x5               g204(.a(new_n295), .b(new_n270), .c(new_n272), .out0(new_n300));
  nano22aa1n03x5               g205(.a(new_n300), .b(new_n297), .c(new_n298), .out0(new_n301));
  norp02aa1n03x5               g206(.a(new_n299), .b(new_n301), .o1(\s[30] ));
  norb02aa1n02x5               g207(.a(new_n295), .b(new_n298), .out0(new_n303));
  aobi12aa1n03x5               g208(.a(new_n303), .b(new_n270), .c(new_n272), .out0(new_n304));
  oao003aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .c(new_n297), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  nano22aa1n03x5               g211(.a(new_n304), .b(new_n305), .c(new_n306), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n303), .b(new_n282), .c(new_n176), .d(new_n269), .o1(new_n308));
  aoi012aa1n02x7               g213(.a(new_n306), .b(new_n308), .c(new_n305), .o1(new_n309));
  norp02aa1n03x5               g214(.a(new_n309), .b(new_n307), .o1(\s[31] ));
  xorb03aa1n02x5               g215(.a(new_n106), .b(\b[2] ), .c(new_n98), .out0(\s[3] ));
  xnrc02aa1n02x5               g216(.a(\b[3] ), .b(\a[4] ), .out0(new_n312));
  oao003aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n106), .carry(new_n313));
  mtn022aa1n02x5               g218(.a(new_n313), .b(new_n107), .sa(new_n312), .o1(\s[4] ));
  xorc02aa1n02x5               g219(.a(\a[5] ), .b(\b[4] ), .out0(new_n315));
  oaoi13aa1n02x5               g220(.a(new_n315), .b(new_n107), .c(new_n99), .d(new_n100), .o1(new_n316));
  oai112aa1n06x5               g221(.a(new_n107), .b(new_n315), .c(new_n100), .d(new_n99), .o1(new_n317));
  norb02aa1n02x5               g222(.a(new_n317), .b(new_n316), .out0(\s[5] ));
  xnbna2aa1n03x5               g223(.a(new_n110), .b(new_n317), .c(new_n114), .out0(\s[6] ));
  nanp02aa1n03x5               g224(.a(new_n317), .b(new_n114), .o1(new_n320));
  ao0022aa1n03x5               g225(.a(new_n320), .b(new_n110), .c(new_n108), .d(new_n121), .o(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi022aa1n03x5               g227(.a(new_n320), .b(new_n110), .c(new_n108), .d(new_n121), .o1(new_n323));
  oao003aa1n03x5               g228(.a(\a[7] ), .b(\b[6] ), .c(new_n323), .carry(new_n324));
  xnrb03aa1n03x5               g229(.a(new_n324), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g230(.a(new_n143), .b(new_n124), .out0(\s[9] ));
endmodule


