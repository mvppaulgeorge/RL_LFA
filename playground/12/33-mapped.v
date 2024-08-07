// Benchmark "adder" written by ABC on Wed Jul 17 18:23:40 2024

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
    new_n118, new_n119, new_n120, new_n122, new_n123, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n130, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n310, new_n312, new_n313, new_n314, new_n317, new_n318, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nand02aa1n06x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nor002aa1d32x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nand22aa1n04x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nona23aa1n09x5               g005(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n101));
  inv040aa1n04x5               g006(.a(new_n101), .o1(new_n102));
  xnrc02aa1n12x5               g007(.a(\b[2] ), .b(\a[3] ), .out0(new_n103));
  nand02aa1n06x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nor042aa1n09x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand02aa1d24x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  oaih12aa1n12x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  oa0022aa1n09x5               g012(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n108));
  oai012aa1d24x5               g013(.a(new_n108), .b(new_n103), .c(new_n107), .o1(new_n109));
  tech160nm_fixnrc02aa1n05x5   g014(.a(\b[4] ), .b(\a[5] ), .out0(new_n110));
  nand42aa1n03x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  orn002aa1n24x5               g016(.a(\a[6] ), .b(\b[5] ), .o(new_n112));
  nand22aa1n12x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nano32aa1n09x5               g018(.a(new_n110), .b(new_n113), .c(new_n111), .d(new_n112), .out0(new_n114));
  nor042aa1n06x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  aob012aa1n12x5               g020(.a(new_n112), .b(new_n115), .c(new_n113), .out0(new_n116));
  oaih12aa1n02x5               g021(.a(new_n98), .b(new_n99), .c(new_n97), .o1(new_n117));
  oaib12aa1n09x5               g022(.a(new_n117), .b(new_n101), .c(new_n116), .out0(new_n118));
  aoi013aa1n09x5               g023(.a(new_n118), .b(new_n109), .c(new_n114), .d(new_n102), .o1(new_n119));
  oaoi03aa1n09x5               g024(.a(\a[9] ), .b(\b[8] ), .c(new_n119), .o1(new_n120));
  xorb03aa1n02x5               g025(.a(new_n120), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  xorc02aa1n12x5               g026(.a(\a[10] ), .b(\b[9] ), .out0(new_n122));
  nanp02aa1n02x5               g027(.a(new_n120), .b(new_n122), .o1(new_n123));
  inv000aa1d42x5               g028(.a(\a[10] ), .o1(new_n124));
  inv000aa1d42x5               g029(.a(\b[9] ), .o1(new_n125));
  nor042aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  oaoi03aa1n03x5               g031(.a(new_n124), .b(new_n125), .c(new_n126), .o1(new_n127));
  tech160nm_fixorc02aa1n03p5x5 g032(.a(\a[11] ), .b(\b[10] ), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n123), .c(new_n127), .out0(\s[11] ));
  and002aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o(new_n130));
  inv020aa1n02x5               g035(.a(new_n130), .o1(new_n131));
  oao003aa1n02x5               g036(.a(new_n124), .b(new_n125), .c(new_n126), .carry(new_n132));
  tech160nm_fixnrc02aa1n04x5   g037(.a(\b[10] ), .b(\a[11] ), .out0(new_n133));
  aoi112aa1n03x5               g038(.a(new_n133), .b(new_n132), .c(new_n120), .d(new_n122), .o1(new_n134));
  tech160nm_fixorc02aa1n03p5x5 g039(.a(\a[12] ), .b(\b[11] ), .out0(new_n135));
  nona22aa1n03x5               g040(.a(new_n131), .b(new_n134), .c(new_n135), .out0(new_n136));
  oai012aa1n02x5               g041(.a(new_n135), .b(new_n134), .c(new_n130), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(new_n136), .b(new_n137), .o1(\s[12] ));
  nand23aa1n03x5               g043(.a(new_n109), .b(new_n114), .c(new_n102), .o1(new_n139));
  aobi12aa1n06x5               g044(.a(new_n117), .b(new_n102), .c(new_n116), .out0(new_n140));
  xnrc02aa1n12x5               g045(.a(\b[8] ), .b(\a[9] ), .out0(new_n141));
  nona23aa1n02x4               g046(.a(new_n135), .b(new_n122), .c(new_n141), .d(new_n133), .out0(new_n142));
  and002aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o(new_n143));
  inv000aa1n02x5               g048(.a(new_n143), .o1(new_n144));
  oa0022aa1n06x5               g049(.a(\a[12] ), .b(\b[11] ), .c(\a[11] ), .d(\b[10] ), .o(new_n145));
  inv000aa1n02x5               g050(.a(new_n145), .o1(new_n146));
  aoai13aa1n03x5               g051(.a(new_n144), .b(new_n146), .c(new_n132), .d(new_n131), .o1(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n142), .c(new_n139), .d(new_n140), .o1(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g054(.a(\a[14] ), .o1(new_n150));
  nor042aa1n12x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  tech160nm_fixorc02aa1n05x5   g056(.a(\a[13] ), .b(\b[12] ), .out0(new_n152));
  aoi012aa1n02x5               g057(.a(new_n151), .b(new_n148), .c(new_n152), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[13] ), .c(new_n150), .out0(\s[14] ));
  inv000aa1d42x5               g059(.a(\a[13] ), .o1(new_n155));
  xroi22aa1d04x5               g060(.a(new_n155), .b(\b[12] ), .c(new_n150), .d(\b[13] ), .out0(new_n156));
  aob012aa1d18x5               g061(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(new_n157));
  oaib12aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(new_n150), .out0(new_n158));
  xorc02aa1n12x5               g063(.a(\a[15] ), .b(\b[14] ), .out0(new_n159));
  aoai13aa1n06x5               g064(.a(new_n159), .b(new_n158), .c(new_n148), .d(new_n156), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n159), .b(new_n158), .c(new_n148), .d(new_n156), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n160), .b(new_n161), .out0(\s[15] ));
  norp02aa1n04x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  inv020aa1n04x5               g068(.a(new_n163), .o1(new_n164));
  xorc02aa1n12x5               g069(.a(\a[16] ), .b(\b[15] ), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n160), .c(new_n164), .out0(\s[16] ));
  nano32aa1n03x7               g071(.a(new_n141), .b(new_n135), .c(new_n122), .d(new_n128), .out0(new_n167));
  xorc02aa1n02x5               g072(.a(\a[14] ), .b(\b[13] ), .out0(new_n168));
  nand22aa1n02x5               g073(.a(new_n168), .b(new_n152), .o1(new_n169));
  nano22aa1n03x7               g074(.a(new_n169), .b(new_n159), .c(new_n165), .out0(new_n170));
  nand02aa1n04x5               g075(.a(new_n167), .b(new_n170), .o1(new_n171));
  oaoi13aa1n06x5               g076(.a(new_n143), .b(new_n145), .c(new_n127), .d(new_n130), .o1(new_n172));
  inv000aa1d42x5               g077(.a(\a[16] ), .o1(new_n173));
  oai112aa1n06x5               g078(.a(new_n157), .b(new_n164), .c(\b[13] ), .d(\a[14] ), .o1(new_n174));
  aoi022aa1d24x5               g079(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n175));
  aboi22aa1n09x5               g080(.a(\b[15] ), .b(new_n173), .c(new_n174), .d(new_n175), .out0(new_n176));
  inv020aa1n03x5               g081(.a(new_n176), .o1(new_n177));
  aoi012aa1n12x5               g082(.a(new_n177), .b(new_n172), .c(new_n170), .o1(new_n178));
  oai012aa1d24x5               g083(.a(new_n178), .b(new_n119), .c(new_n171), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g085(.a(\a[18] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\a[17] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[16] ), .o1(new_n183));
  oaoi03aa1n02x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  xroi22aa1d06x4               g090(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n186));
  nor002aa1n03x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  aoi112aa1n09x5               g092(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n188));
  nor042aa1n06x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  inv000aa1d42x5               g094(.a(new_n189), .o1(new_n190));
  nor002aa1d32x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  nand02aa1n04x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  norb02aa1n06x5               g097(.a(new_n192), .b(new_n191), .out0(new_n193));
  aoai13aa1n06x5               g098(.a(new_n193), .b(new_n190), .c(new_n179), .d(new_n186), .o1(new_n194));
  aoi112aa1n02x5               g099(.a(new_n193), .b(new_n190), .c(new_n179), .d(new_n186), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n194), .b(new_n195), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n12x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nand02aa1n08x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  norb02aa1n06x4               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  nona22aa1n03x5               g105(.a(new_n194), .b(new_n200), .c(new_n191), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n191), .o1(new_n202));
  aobi12aa1n06x5               g107(.a(new_n200), .b(new_n194), .c(new_n202), .out0(new_n203));
  norb02aa1n03x4               g108(.a(new_n201), .b(new_n203), .out0(\s[20] ));
  nona23aa1n09x5               g109(.a(new_n199), .b(new_n192), .c(new_n191), .d(new_n198), .out0(new_n205));
  inv000aa1n02x5               g110(.a(new_n205), .o1(new_n206));
  nand02aa1n04x5               g111(.a(new_n186), .b(new_n206), .o1(new_n207));
  inv040aa1n06x5               g112(.a(new_n207), .o1(new_n208));
  tech160nm_fioai012aa1n03p5x5 g113(.a(new_n199), .b(new_n198), .c(new_n191), .o1(new_n209));
  oai012aa1n06x5               g114(.a(new_n209), .b(new_n205), .c(new_n189), .o1(new_n210));
  xorc02aa1n12x5               g115(.a(\a[21] ), .b(\b[20] ), .out0(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n210), .c(new_n179), .d(new_n208), .o1(new_n212));
  aoi112aa1n02x5               g117(.a(new_n211), .b(new_n210), .c(new_n179), .d(new_n208), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n212), .b(new_n213), .out0(\s[21] ));
  nor042aa1n03x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  xorc02aa1n12x5               g120(.a(\a[22] ), .b(\b[21] ), .out0(new_n216));
  nona22aa1n06x5               g121(.a(new_n212), .b(new_n216), .c(new_n215), .out0(new_n217));
  inv020aa1n02x5               g122(.a(new_n215), .o1(new_n218));
  aobi12aa1n06x5               g123(.a(new_n216), .b(new_n212), .c(new_n218), .out0(new_n219));
  norb02aa1n03x4               g124(.a(new_n217), .b(new_n219), .out0(\s[22] ));
  nand42aa1n04x5               g125(.a(new_n216), .b(new_n211), .o1(new_n221));
  nano22aa1n02x4               g126(.a(new_n221), .b(new_n186), .c(new_n206), .out0(new_n222));
  oai112aa1n03x5               g127(.a(new_n193), .b(new_n200), .c(new_n188), .d(new_n187), .o1(new_n223));
  tech160nm_fioaoi03aa1n02p5x5 g128(.a(\a[22] ), .b(\b[21] ), .c(new_n218), .o1(new_n224));
  inv000aa1n02x5               g129(.a(new_n224), .o1(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n221), .c(new_n223), .d(new_n209), .o1(new_n226));
  xorc02aa1n12x5               g131(.a(\a[23] ), .b(\b[22] ), .out0(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n226), .c(new_n179), .d(new_n222), .o1(new_n228));
  aoi112aa1n02x5               g133(.a(new_n227), .b(new_n226), .c(new_n179), .d(new_n222), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n228), .b(new_n229), .out0(\s[23] ));
  nor042aa1n03x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  xorc02aa1n12x5               g136(.a(\a[24] ), .b(\b[23] ), .out0(new_n232));
  nona22aa1n06x5               g137(.a(new_n228), .b(new_n232), .c(new_n231), .out0(new_n233));
  inv000aa1n02x5               g138(.a(new_n231), .o1(new_n234));
  aobi12aa1n06x5               g139(.a(new_n232), .b(new_n228), .c(new_n234), .out0(new_n235));
  norb02aa1n03x4               g140(.a(new_n233), .b(new_n235), .out0(\s[24] ));
  nanp02aa1n02x5               g141(.a(new_n232), .b(new_n227), .o1(new_n237));
  nor043aa1n02x5               g142(.a(new_n207), .b(new_n221), .c(new_n237), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n221), .o1(new_n239));
  inv000aa1n02x5               g144(.a(new_n237), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n224), .c(new_n210), .d(new_n239), .o1(new_n241));
  oaoi03aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .c(new_n234), .o1(new_n242));
  inv000aa1n02x5               g147(.a(new_n242), .o1(new_n243));
  nand22aa1n02x5               g148(.a(new_n241), .b(new_n243), .o1(new_n244));
  xorc02aa1n12x5               g149(.a(\a[25] ), .b(\b[24] ), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n244), .c(new_n179), .d(new_n238), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(new_n245), .b(new_n244), .c(new_n179), .d(new_n238), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n246), .b(new_n247), .out0(\s[25] ));
  nor042aa1n03x5               g153(.a(\b[24] ), .b(\a[25] ), .o1(new_n249));
  xorc02aa1n12x5               g154(.a(\a[26] ), .b(\b[25] ), .out0(new_n250));
  nona22aa1n06x5               g155(.a(new_n246), .b(new_n250), .c(new_n249), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n249), .o1(new_n252));
  aobi12aa1n06x5               g157(.a(new_n250), .b(new_n246), .c(new_n252), .out0(new_n253));
  norb02aa1n03x4               g158(.a(new_n251), .b(new_n253), .out0(\s[26] ));
  nanp02aa1n03x5               g159(.a(new_n139), .b(new_n140), .o1(new_n255));
  inv000aa1d42x5               g160(.a(\a[15] ), .o1(new_n256));
  xroi22aa1d04x5               g161(.a(new_n256), .b(\b[14] ), .c(new_n173), .d(\b[15] ), .out0(new_n257));
  nano22aa1n03x7               g162(.a(new_n142), .b(new_n156), .c(new_n257), .out0(new_n258));
  nanp02aa1n02x5               g163(.a(new_n257), .b(new_n156), .o1(new_n259));
  oai012aa1n03x5               g164(.a(new_n176), .b(new_n147), .c(new_n259), .o1(new_n260));
  nand02aa1d04x5               g165(.a(new_n250), .b(new_n245), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  nano32aa1n03x7               g167(.a(new_n207), .b(new_n262), .c(new_n239), .d(new_n240), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n260), .c(new_n255), .d(new_n258), .o1(new_n264));
  aoai13aa1n06x5               g169(.a(new_n262), .b(new_n242), .c(new_n226), .d(new_n240), .o1(new_n265));
  oao003aa1n02x5               g170(.a(\a[26] ), .b(\b[25] ), .c(new_n252), .carry(new_n266));
  xorc02aa1n12x5               g171(.a(\a[27] ), .b(\b[26] ), .out0(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  aoi013aa1n06x4               g173(.a(new_n268), .b(new_n264), .c(new_n265), .d(new_n266), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n266), .b(new_n261), .c(new_n241), .d(new_n243), .o1(new_n270));
  aoi112aa1n02x5               g175(.a(new_n270), .b(new_n267), .c(new_n179), .d(new_n263), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n269), .b(new_n271), .o1(\s[27] ));
  nor042aa1n03x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  xnrc02aa1n12x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  nano22aa1n03x5               g180(.a(new_n269), .b(new_n274), .c(new_n275), .out0(new_n276));
  nona32aa1n03x5               g181(.a(new_n208), .b(new_n261), .c(new_n237), .d(new_n221), .out0(new_n277));
  oaoi13aa1n06x5               g182(.a(new_n277), .b(new_n178), .c(new_n119), .d(new_n171), .o1(new_n278));
  oaih12aa1n02x5               g183(.a(new_n267), .b(new_n270), .c(new_n278), .o1(new_n279));
  tech160nm_fiaoi012aa1n02p5x5 g184(.a(new_n275), .b(new_n279), .c(new_n274), .o1(new_n280));
  norp02aa1n03x5               g185(.a(new_n280), .b(new_n276), .o1(\s[28] ));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  norb02aa1n02x5               g187(.a(new_n267), .b(new_n275), .out0(new_n283));
  oaih12aa1n02x5               g188(.a(new_n283), .b(new_n270), .c(new_n278), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n274), .carry(new_n285));
  aoi012aa1n03x5               g190(.a(new_n282), .b(new_n284), .c(new_n285), .o1(new_n286));
  inv000aa1n02x5               g191(.a(new_n283), .o1(new_n287));
  aoi013aa1n02x4               g192(.a(new_n287), .b(new_n264), .c(new_n265), .d(new_n266), .o1(new_n288));
  nano22aa1n03x5               g193(.a(new_n288), .b(new_n282), .c(new_n285), .out0(new_n289));
  nor002aa1n02x5               g194(.a(new_n286), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g196(.a(\b[29] ), .b(\a[30] ), .out0(new_n292));
  norb03aa1n12x5               g197(.a(new_n267), .b(new_n282), .c(new_n275), .out0(new_n293));
  oaih12aa1n02x5               g198(.a(new_n293), .b(new_n270), .c(new_n278), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n295));
  tech160nm_fiaoi012aa1n02p5x5 g200(.a(new_n292), .b(new_n294), .c(new_n295), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n293), .o1(new_n297));
  aoi013aa1n03x5               g202(.a(new_n297), .b(new_n264), .c(new_n265), .d(new_n266), .o1(new_n298));
  nano22aa1n03x5               g203(.a(new_n298), .b(new_n292), .c(new_n295), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n296), .b(new_n299), .o1(\s[30] ));
  norb02aa1n02x5               g205(.a(new_n293), .b(new_n292), .out0(new_n301));
  inv000aa1n02x5               g206(.a(new_n301), .o1(new_n302));
  aoi013aa1n03x5               g207(.a(new_n302), .b(new_n264), .c(new_n265), .d(new_n266), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n295), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nano22aa1n03x5               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  oaih12aa1n02x5               g211(.a(new_n301), .b(new_n270), .c(new_n278), .o1(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n03x5               g213(.a(new_n308), .b(new_n306), .o1(\s[31] ));
  inv000aa1d42x5               g214(.a(\a[3] ), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n107), .b(\b[2] ), .c(new_n310), .out0(\s[3] ));
  norp02aa1n02x5               g216(.a(new_n103), .b(new_n107), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[4] ), .b(\b[3] ), .out0(new_n313));
  aoib12aa1n02x5               g218(.a(new_n313), .b(new_n310), .c(\b[2] ), .out0(new_n314));
  aboi22aa1n03x5               g219(.a(new_n312), .b(new_n314), .c(new_n109), .d(new_n313), .out0(\s[4] ));
  xnbna2aa1n03x5               g220(.a(new_n110), .b(new_n109), .c(new_n111), .out0(\s[5] ));
  nanp02aa1n02x5               g221(.a(new_n109), .b(new_n111), .o1(new_n317));
  oaoi03aa1n02x5               g222(.a(\a[5] ), .b(\b[4] ), .c(new_n317), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g224(.a(new_n116), .b(new_n109), .c(new_n114), .o(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g226(.a(new_n99), .b(new_n320), .c(new_n100), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g228(.a(new_n141), .b(new_n139), .c(new_n140), .out0(\s[9] ));
endmodule


