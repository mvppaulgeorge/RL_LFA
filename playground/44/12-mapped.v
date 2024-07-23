// Benchmark "adder" written by ABC on Thu Jul 18 10:35:43 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n123, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n177, new_n178, new_n179, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n312, new_n315, new_n316, new_n318, new_n320, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n16x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nand02aa1n10x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nor022aa1n16x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nand42aa1n04x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nona23aa1d18x5               g005(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n101));
  xnrc02aa1n02x5               g006(.a(\b[5] ), .b(\a[6] ), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[4] ), .b(\a[5] ), .out0(new_n103));
  nor043aa1n03x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  nor042aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand02aa1d06x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  nand22aa1n04x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  tech160nm_fiaoi012aa1n04x5   g012(.a(new_n105), .b(new_n106), .c(new_n107), .o1(new_n108));
  xnrc02aa1n02x5               g013(.a(\b[3] ), .b(\a[4] ), .out0(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[2] ), .b(\a[3] ), .out0(new_n110));
  oai022aa1n02x5               g015(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n111));
  aob012aa1n02x5               g016(.a(new_n111), .b(\b[3] ), .c(\a[4] ), .out0(new_n112));
  oai013aa1n06x5               g017(.a(new_n112), .b(new_n109), .c(new_n110), .d(new_n108), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(new_n99), .b(new_n98), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\a[6] ), .o1(new_n115));
  inv000aa1d42x5               g020(.a(\b[5] ), .o1(new_n116));
  aoi112aa1n09x5               g021(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n117));
  aoi012aa1n12x5               g022(.a(new_n117), .b(new_n115), .c(new_n116), .o1(new_n118));
  oai122aa1n12x5               g023(.a(new_n114), .b(new_n101), .c(new_n118), .d(\b[7] ), .e(\a[8] ), .o1(new_n119));
  tech160nm_fiaoi012aa1n05x5   g024(.a(new_n119), .b(new_n113), .c(new_n104), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[9] ), .b(\b[8] ), .c(new_n120), .o1(new_n121));
  xorb03aa1n02x5               g026(.a(new_n121), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1n04x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  tech160nm_finand02aa1n03p5x5 g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nor042aa1n03x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1n08x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nano23aa1n03x7               g031(.a(new_n123), .b(new_n125), .c(new_n126), .d(new_n124), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n119), .c(new_n113), .d(new_n104), .o1(new_n128));
  aoi012aa1n12x5               g033(.a(new_n125), .b(new_n123), .c(new_n126), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(new_n128), .b(new_n129), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g036(.a(\a[11] ), .o1(new_n132));
  inv000aa1d42x5               g037(.a(\b[10] ), .o1(new_n133));
  oaoi03aa1n02x5               g038(.a(new_n132), .b(new_n133), .c(new_n130), .o1(new_n134));
  xnrb03aa1n02x5               g039(.a(new_n134), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor022aa1n06x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand02aa1n03x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nor022aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand22aa1n04x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nona23aa1n02x4               g044(.a(new_n139), .b(new_n137), .c(new_n136), .d(new_n138), .out0(new_n140));
  oa0012aa1n03x5               g045(.a(new_n139), .b(new_n138), .c(new_n136), .o(new_n141));
  inv020aa1n03x5               g046(.a(new_n141), .o1(new_n142));
  tech160nm_fioai012aa1n04x5   g047(.a(new_n142), .b(new_n140), .c(new_n129), .o1(new_n143));
  nano23aa1n06x5               g048(.a(new_n136), .b(new_n138), .c(new_n139), .d(new_n137), .out0(new_n144));
  nano22aa1n03x7               g049(.a(new_n120), .b(new_n127), .c(new_n144), .out0(new_n145));
  tech160nm_fixnrc02aa1n02p5x5 g050(.a(\b[12] ), .b(\a[13] ), .out0(new_n146));
  oabi12aa1n03x5               g051(.a(new_n146), .b(new_n145), .c(new_n143), .out0(new_n147));
  nona22aa1n02x4               g052(.a(new_n146), .b(new_n145), .c(new_n143), .out0(new_n148));
  and002aa1n02x5               g053(.a(new_n148), .b(new_n147), .o(\s[13] ));
  orn002aa1n24x5               g054(.a(\a[13] ), .b(\b[12] ), .o(new_n150));
  xnrc02aa1n06x5               g055(.a(\b[13] ), .b(\a[14] ), .out0(new_n151));
  xobna2aa1n03x5               g056(.a(new_n151), .b(new_n147), .c(new_n150), .out0(\s[14] ));
  nor042aa1n03x5               g057(.a(new_n151), .b(new_n146), .o1(new_n153));
  nano32aa1n03x7               g058(.a(new_n120), .b(new_n153), .c(new_n127), .d(new_n144), .out0(new_n154));
  inv020aa1n02x5               g059(.a(new_n129), .o1(new_n155));
  aoai13aa1n06x5               g060(.a(new_n153), .b(new_n141), .c(new_n144), .d(new_n155), .o1(new_n156));
  tech160nm_fioaoi03aa1n03p5x5 g061(.a(\a[14] ), .b(\b[13] ), .c(new_n150), .o1(new_n157));
  inv000aa1n02x5               g062(.a(new_n157), .o1(new_n158));
  nand22aa1n03x5               g063(.a(new_n156), .b(new_n158), .o1(new_n159));
  xnrc02aa1n12x5               g064(.a(\b[14] ), .b(\a[15] ), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  tech160nm_fioai012aa1n05x5   g066(.a(new_n161), .b(new_n154), .c(new_n159), .o1(new_n162));
  nano32aa1n02x4               g067(.a(new_n154), .b(new_n160), .c(new_n156), .d(new_n158), .out0(new_n163));
  norb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(\s[15] ));
  nor042aa1n06x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  xnrc02aa1n12x5               g071(.a(\b[15] ), .b(\a[16] ), .out0(new_n167));
  xobna2aa1n03x5               g072(.a(new_n167), .b(new_n162), .c(new_n166), .out0(\s[16] ));
  nand02aa1n06x5               g073(.a(new_n144), .b(new_n127), .o1(new_n169));
  nor042aa1d18x5               g074(.a(new_n167), .b(new_n160), .o1(new_n170));
  nano22aa1d15x5               g075(.a(new_n169), .b(new_n153), .c(new_n170), .out0(new_n171));
  aoai13aa1n12x5               g076(.a(new_n171), .b(new_n119), .c(new_n113), .d(new_n104), .o1(new_n172));
  aoai13aa1n04x5               g077(.a(new_n170), .b(new_n157), .c(new_n143), .d(new_n153), .o1(new_n173));
  oao003aa1n02x5               g078(.a(\a[16] ), .b(\b[15] ), .c(new_n166), .carry(new_n174));
  nand23aa1d12x5               g079(.a(new_n172), .b(new_n173), .c(new_n174), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1d18x5               g081(.a(\b[16] ), .b(\a[17] ), .o1(new_n177));
  nanp02aa1n12x5               g082(.a(\b[16] ), .b(\a[17] ), .o1(new_n178));
  tech160nm_fiaoi012aa1n05x5   g083(.a(new_n177), .b(new_n175), .c(new_n178), .o1(new_n179));
  xnrb03aa1n03x5               g084(.a(new_n179), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  aobi12aa1n06x5               g085(.a(new_n174), .b(new_n159), .c(new_n170), .out0(new_n181));
  nor042aa1d18x5               g086(.a(\b[17] ), .b(\a[18] ), .o1(new_n182));
  nand02aa1d28x5               g087(.a(\b[17] ), .b(\a[18] ), .o1(new_n183));
  nano23aa1d15x5               g088(.a(new_n177), .b(new_n182), .c(new_n183), .d(new_n178), .out0(new_n184));
  inv000aa1d42x5               g089(.a(new_n184), .o1(new_n185));
  aoi012aa1d18x5               g090(.a(new_n182), .b(new_n177), .c(new_n183), .o1(new_n186));
  aoai13aa1n04x5               g091(.a(new_n186), .b(new_n185), .c(new_n181), .d(new_n172), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g093(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nand22aa1n06x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  nor002aa1n04x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  nand02aa1d04x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoi112aa1n02x5               g100(.a(new_n190), .b(new_n195), .c(new_n187), .d(new_n192), .o1(new_n196));
  inv040aa1n08x5               g101(.a(new_n190), .o1(new_n197));
  inv000aa1n02x5               g102(.a(new_n186), .o1(new_n198));
  aoai13aa1n06x5               g103(.a(new_n192), .b(new_n198), .c(new_n175), .d(new_n184), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n195), .o1(new_n200));
  tech160nm_fiaoi012aa1n05x5   g105(.a(new_n200), .b(new_n199), .c(new_n197), .o1(new_n201));
  nor042aa1n03x5               g106(.a(new_n201), .b(new_n196), .o1(\s[20] ));
  nano23aa1n06x5               g107(.a(new_n190), .b(new_n193), .c(new_n194), .d(new_n191), .out0(new_n203));
  nand02aa1d08x5               g108(.a(new_n203), .b(new_n184), .o1(new_n204));
  nona23aa1n12x5               g109(.a(new_n194), .b(new_n191), .c(new_n190), .d(new_n193), .out0(new_n205));
  oaoi03aa1n12x5               g110(.a(\a[20] ), .b(\b[19] ), .c(new_n197), .o1(new_n206));
  inv040aa1n02x5               g111(.a(new_n206), .o1(new_n207));
  oai012aa1d24x5               g112(.a(new_n207), .b(new_n205), .c(new_n186), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoai13aa1n04x5               g114(.a(new_n209), .b(new_n204), .c(new_n181), .d(new_n172), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  tech160nm_fixorc02aa1n03p5x5 g118(.a(\a[22] ), .b(\b[21] ), .out0(new_n214));
  aoi112aa1n02x5               g119(.a(new_n212), .b(new_n214), .c(new_n210), .d(new_n213), .o1(new_n215));
  inv040aa1n08x5               g120(.a(new_n212), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n204), .o1(new_n217));
  aoai13aa1n06x5               g122(.a(new_n213), .b(new_n208), .c(new_n175), .d(new_n217), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n214), .o1(new_n219));
  tech160nm_fiaoi012aa1n05x5   g124(.a(new_n219), .b(new_n218), .c(new_n216), .o1(new_n220));
  nor002aa1n02x5               g125(.a(new_n220), .b(new_n215), .o1(\s[22] ));
  nano22aa1n02x4               g126(.a(new_n204), .b(new_n213), .c(new_n214), .out0(new_n222));
  inv000aa1n02x5               g127(.a(new_n222), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\a[21] ), .o1(new_n224));
  inv020aa1n04x5               g129(.a(\a[22] ), .o1(new_n225));
  xroi22aa1d06x4               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  oaoi03aa1n03x5               g131(.a(\a[22] ), .b(\b[21] ), .c(new_n216), .o1(new_n227));
  aoi012aa1d18x5               g132(.a(new_n227), .b(new_n208), .c(new_n226), .o1(new_n228));
  aoai13aa1n04x5               g133(.a(new_n228), .b(new_n223), .c(new_n181), .d(new_n172), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  tech160nm_fixorc02aa1n03p5x5 g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  xorc02aa1n12x5               g137(.a(\a[24] ), .b(\b[23] ), .out0(new_n233));
  aoi112aa1n03x4               g138(.a(new_n231), .b(new_n233), .c(new_n229), .d(new_n232), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n231), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n228), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n232), .b(new_n236), .c(new_n175), .d(new_n222), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n233), .o1(new_n238));
  tech160nm_fiaoi012aa1n02p5x5 g143(.a(new_n238), .b(new_n237), .c(new_n235), .o1(new_n239));
  nor002aa1n02x5               g144(.a(new_n239), .b(new_n234), .o1(\s[24] ));
  and002aa1n02x5               g145(.a(new_n233), .b(new_n232), .o(new_n241));
  inv040aa1n02x5               g146(.a(new_n241), .o1(new_n242));
  nano32aa1n02x4               g147(.a(new_n242), .b(new_n226), .c(new_n203), .d(new_n184), .out0(new_n243));
  inv000aa1n02x5               g148(.a(new_n243), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n226), .b(new_n206), .c(new_n203), .d(new_n198), .o1(new_n245));
  inv030aa1n03x5               g150(.a(new_n227), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(\b[23] ), .b(\a[24] ), .o1(new_n247));
  oai022aa1n02x5               g152(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n248));
  nanp02aa1n02x5               g153(.a(new_n248), .b(new_n247), .o1(new_n249));
  aoai13aa1n12x5               g154(.a(new_n249), .b(new_n242), .c(new_n245), .d(new_n246), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n244), .c(new_n181), .d(new_n172), .o1(new_n252));
  xorb03aa1n02x5               g157(.a(new_n252), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  tech160nm_fixorc02aa1n04x5   g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  xorc02aa1n12x5               g160(.a(\a[26] ), .b(\b[25] ), .out0(new_n256));
  aoi112aa1n03x4               g161(.a(new_n254), .b(new_n256), .c(new_n252), .d(new_n255), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n254), .o1(new_n258));
  aoai13aa1n03x5               g163(.a(new_n255), .b(new_n250), .c(new_n175), .d(new_n243), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n256), .o1(new_n260));
  aoi012aa1n03x5               g165(.a(new_n260), .b(new_n259), .c(new_n258), .o1(new_n261));
  nor002aa1n02x5               g166(.a(new_n261), .b(new_n257), .o1(\s[26] ));
  nanp02aa1n02x5               g167(.a(new_n113), .b(new_n104), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n119), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(new_n264), .b(new_n263), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n170), .o1(new_n266));
  aoai13aa1n02x5               g171(.a(new_n174), .b(new_n266), .c(new_n156), .d(new_n158), .o1(new_n267));
  and002aa1n02x5               g172(.a(new_n256), .b(new_n255), .o(new_n268));
  inv020aa1n02x5               g173(.a(new_n268), .o1(new_n269));
  nano23aa1n03x7               g174(.a(new_n204), .b(new_n269), .c(new_n241), .d(new_n226), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n267), .c(new_n265), .d(new_n171), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .c(new_n258), .carry(new_n272));
  aobi12aa1n12x5               g177(.a(new_n272), .b(new_n250), .c(new_n268), .out0(new_n273));
  xorc02aa1n02x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n271), .c(new_n273), .out0(\s[27] ));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  inv040aa1n03x5               g181(.a(new_n276), .o1(new_n277));
  aobi12aa1n02x7               g182(.a(new_n274), .b(new_n271), .c(new_n273), .out0(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .out0(new_n279));
  nano22aa1n02x4               g184(.a(new_n278), .b(new_n277), .c(new_n279), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n241), .b(new_n227), .c(new_n208), .d(new_n226), .o1(new_n281));
  aoai13aa1n04x5               g186(.a(new_n272), .b(new_n269), .c(new_n281), .d(new_n249), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n274), .b(new_n282), .c(new_n175), .d(new_n270), .o1(new_n283));
  tech160nm_fiaoi012aa1n02p5x5 g188(.a(new_n279), .b(new_n283), .c(new_n277), .o1(new_n284));
  norp02aa1n03x5               g189(.a(new_n284), .b(new_n280), .o1(\s[28] ));
  norb02aa1n02x5               g190(.a(new_n274), .b(new_n279), .out0(new_n286));
  aobi12aa1n02x7               g191(.a(new_n286), .b(new_n271), .c(new_n273), .out0(new_n287));
  oao003aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[28] ), .b(\a[29] ), .out0(new_n289));
  nano22aa1n03x5               g194(.a(new_n287), .b(new_n288), .c(new_n289), .out0(new_n290));
  aoai13aa1n03x5               g195(.a(new_n286), .b(new_n282), .c(new_n175), .d(new_n270), .o1(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n289), .b(new_n291), .c(new_n288), .o1(new_n292));
  norp02aa1n03x5               g197(.a(new_n292), .b(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g199(.a(new_n274), .b(new_n289), .c(new_n279), .out0(new_n295));
  aobi12aa1n02x7               g200(.a(new_n295), .b(new_n271), .c(new_n273), .out0(new_n296));
  oao003aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n288), .carry(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[29] ), .b(\a[30] ), .out0(new_n298));
  nano22aa1n02x4               g203(.a(new_n296), .b(new_n297), .c(new_n298), .out0(new_n299));
  aoai13aa1n03x5               g204(.a(new_n295), .b(new_n282), .c(new_n175), .d(new_n270), .o1(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n298), .b(new_n300), .c(new_n297), .o1(new_n301));
  norp02aa1n03x5               g206(.a(new_n301), .b(new_n299), .o1(\s[30] ));
  norb02aa1n02x5               g207(.a(new_n295), .b(new_n298), .out0(new_n303));
  aobi12aa1n02x7               g208(.a(new_n303), .b(new_n271), .c(new_n273), .out0(new_n304));
  oao003aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .c(new_n297), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  nano22aa1n02x4               g211(.a(new_n304), .b(new_n305), .c(new_n306), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n303), .b(new_n282), .c(new_n175), .d(new_n270), .o1(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n306), .b(new_n308), .c(new_n305), .o1(new_n309));
  norp02aa1n03x5               g214(.a(new_n309), .b(new_n307), .o1(\s[31] ));
  xnrb03aa1n02x5               g215(.a(new_n108), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g216(.a(\a[3] ), .b(\b[2] ), .c(new_n108), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g218(.a(new_n113), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g219(.a(new_n103), .b(new_n113), .out0(new_n315));
  oai012aa1n02x5               g220(.a(new_n315), .b(\b[4] ), .c(\a[5] ), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g222(.a(new_n115), .b(new_n116), .c(new_n316), .carry(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g224(.a(new_n115), .b(new_n116), .c(new_n316), .o1(new_n320));
  oaoi03aa1n02x5               g225(.a(\a[7] ), .b(\b[6] ), .c(new_n320), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g227(.a(new_n120), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


