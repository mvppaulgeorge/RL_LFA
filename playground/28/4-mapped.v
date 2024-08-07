// Benchmark "adder" written by ABC on Thu Jul 18 02:18:32 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n316, new_n317, new_n318, new_n320;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nand02aa1d16x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  norb02aa1n12x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor042aa1d18x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nand22aa1n12x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  norb02aa1n06x5               g006(.a(new_n101), .b(new_n100), .out0(new_n102));
  norp02aa1n24x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  nona23aa1n06x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  nano22aa1n06x5               g012(.a(new_n107), .b(new_n99), .c(new_n102), .out0(new_n108));
  nor002aa1n12x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  nand02aa1n04x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nor022aa1n16x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nand42aa1n03x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nona23aa1n09x5               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  nand42aa1n02x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  nand22aa1n04x5               g019(.a(\b[0] ), .b(\a[1] ), .o1(new_n115));
  nor042aa1n04x5               g020(.a(\b[1] ), .b(\a[2] ), .o1(new_n116));
  oaih12aa1n12x5               g021(.a(new_n114), .b(new_n116), .c(new_n115), .o1(new_n117));
  tech160nm_fioai012aa1n03p5x5 g022(.a(new_n110), .b(new_n111), .c(new_n109), .o1(new_n118));
  oai012aa1d24x5               g023(.a(new_n118), .b(new_n113), .c(new_n117), .o1(new_n119));
  aoi112aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  aoi112aa1n09x5               g025(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n121));
  oai112aa1n02x7               g026(.a(new_n102), .b(new_n99), .c(new_n121), .d(new_n103), .o1(new_n122));
  nona22aa1n02x4               g027(.a(new_n122), .b(new_n120), .c(new_n97), .out0(new_n123));
  aoi012aa1n02x5               g028(.a(new_n123), .b(new_n119), .c(new_n108), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(\a[9] ), .b(\b[8] ), .c(new_n124), .o1(new_n125));
  xorb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1n04x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nor042aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nano23aa1n06x5               g035(.a(new_n127), .b(new_n129), .c(new_n130), .d(new_n128), .out0(new_n131));
  aoai13aa1n06x5               g036(.a(new_n131), .b(new_n123), .c(new_n119), .d(new_n108), .o1(new_n132));
  tech160nm_fiao0012aa1n02p5x5 g037(.a(new_n127), .b(new_n129), .c(new_n128), .o(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nor002aa1n12x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1d28x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n06x4               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n132), .c(new_n134), .out0(\s[11] ));
  nanp02aa1n02x5               g043(.a(new_n132), .b(new_n134), .o1(new_n139));
  aoi012aa1n02x5               g044(.a(new_n135), .b(new_n139), .c(new_n136), .o1(new_n140));
  xnrb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor002aa1n16x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand42aa1n16x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nano23aa1d15x5               g048(.a(new_n135), .b(new_n142), .c(new_n143), .d(new_n136), .out0(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  aoi112aa1n02x7               g050(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n146));
  aoi112aa1n09x5               g051(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n147));
  norb02aa1n06x4               g052(.a(new_n143), .b(new_n142), .out0(new_n148));
  oai112aa1n06x5               g053(.a(new_n137), .b(new_n148), .c(new_n147), .d(new_n127), .o1(new_n149));
  nona22aa1d24x5               g054(.a(new_n149), .b(new_n146), .c(new_n142), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nor042aa1n06x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand22aa1n04x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(new_n154));
  oaoi13aa1n06x5               g059(.a(new_n154), .b(new_n151), .c(new_n132), .d(new_n145), .o1(new_n155));
  oai112aa1n02x5               g060(.a(new_n151), .b(new_n154), .c(new_n132), .d(new_n145), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(\s[13] ));
  nor042aa1n04x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand02aa1d06x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanb02aa1n02x5               g064(.a(new_n158), .b(new_n159), .out0(new_n160));
  norb03aa1n02x5               g065(.a(new_n160), .b(new_n155), .c(new_n152), .out0(new_n161));
  oab012aa1n02x4               g066(.a(new_n160), .b(new_n155), .c(new_n152), .out0(new_n162));
  norp02aa1n02x5               g067(.a(new_n162), .b(new_n161), .o1(\s[14] ));
  nano23aa1n09x5               g068(.a(new_n152), .b(new_n158), .c(new_n159), .d(new_n153), .out0(new_n164));
  tech160nm_fiaoi012aa1n05x5   g069(.a(new_n158), .b(new_n152), .c(new_n159), .o1(new_n165));
  aobi12aa1n03x5               g070(.a(new_n165), .b(new_n150), .c(new_n164), .out0(new_n166));
  nano32aa1n03x7               g071(.a(new_n124), .b(new_n164), .c(new_n131), .d(new_n144), .out0(new_n167));
  norp02aa1n04x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1d28x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n06x4               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  oaib12aa1n06x5               g075(.a(new_n170), .b(new_n167), .c(new_n166), .out0(new_n171));
  norb03aa1n02x5               g076(.a(new_n166), .b(new_n167), .c(new_n170), .out0(new_n172));
  norb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(\s[15] ));
  nor042aa1n06x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand42aa1n10x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  norb02aa1n12x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  nona22aa1n02x4               g081(.a(new_n171), .b(new_n176), .c(new_n168), .out0(new_n177));
  inv000aa1d42x5               g082(.a(new_n176), .o1(new_n178));
  oaoi13aa1n06x5               g083(.a(new_n178), .b(new_n171), .c(\a[15] ), .d(\b[14] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n177), .b(new_n179), .out0(\s[16] ));
  nano23aa1n03x5               g085(.a(new_n168), .b(new_n174), .c(new_n175), .d(new_n169), .out0(new_n181));
  nand22aa1n03x5               g086(.a(new_n181), .b(new_n164), .o1(new_n182));
  nano22aa1d15x5               g087(.a(new_n182), .b(new_n131), .c(new_n144), .out0(new_n183));
  aoai13aa1n12x5               g088(.a(new_n183), .b(new_n123), .c(new_n119), .d(new_n108), .o1(new_n184));
  nona23aa1n02x5               g089(.a(new_n159), .b(new_n153), .c(new_n152), .d(new_n158), .out0(new_n185));
  nano22aa1n03x7               g090(.a(new_n185), .b(new_n170), .c(new_n176), .out0(new_n186));
  aoi112aa1n02x5               g091(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n187));
  nanb03aa1n03x5               g092(.a(new_n165), .b(new_n176), .c(new_n170), .out0(new_n188));
  nona22aa1n12x5               g093(.a(new_n188), .b(new_n187), .c(new_n174), .out0(new_n189));
  aoi012aa1d24x5               g094(.a(new_n189), .b(new_n150), .c(new_n186), .o1(new_n190));
  nand02aa1d08x5               g095(.a(new_n184), .b(new_n190), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g097(.a(\a[18] ), .o1(new_n193));
  inv030aa1d32x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[16] ), .o1(new_n195));
  oaoi03aa1n03x5               g100(.a(new_n194), .b(new_n195), .c(new_n191), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n193), .out0(\s[18] ));
  xroi22aa1d06x4               g102(.a(new_n194), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\b[17] ), .o1(new_n200));
  norp02aa1n02x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  oao003aa1n02x5               g106(.a(new_n193), .b(new_n200), .c(new_n201), .carry(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  aoai13aa1n04x5               g108(.a(new_n203), .b(new_n199), .c(new_n184), .d(new_n190), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand02aa1n20x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nor042aa1n09x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand02aa1d16x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  norb02aa1n12x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  aoi112aa1n02x5               g116(.a(new_n207), .b(new_n211), .c(new_n204), .d(new_n208), .o1(new_n212));
  aoai13aa1n03x5               g117(.a(new_n211), .b(new_n207), .c(new_n204), .d(new_n208), .o1(new_n213));
  norb02aa1n02x7               g118(.a(new_n213), .b(new_n212), .out0(\s[20] ));
  nano23aa1n02x5               g119(.a(new_n207), .b(new_n209), .c(new_n210), .d(new_n208), .out0(new_n215));
  nand02aa1n02x5               g120(.a(new_n198), .b(new_n215), .o1(new_n216));
  aoi112aa1n09x5               g121(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n217));
  nor042aa1n02x5               g122(.a(\b[17] ), .b(\a[18] ), .o1(new_n218));
  aoi112aa1n09x5               g123(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n219));
  norb02aa1n12x5               g124(.a(new_n208), .b(new_n207), .out0(new_n220));
  oai112aa1n06x5               g125(.a(new_n220), .b(new_n211), .c(new_n219), .d(new_n218), .o1(new_n221));
  nona22aa1d18x5               g126(.a(new_n221), .b(new_n217), .c(new_n209), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n04x5               g128(.a(new_n223), .b(new_n216), .c(new_n184), .d(new_n190), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n16x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  nand02aa1n03x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  norb02aa1n02x7               g132(.a(new_n227), .b(new_n226), .out0(new_n228));
  nor042aa1n04x5               g133(.a(\b[21] ), .b(\a[22] ), .o1(new_n229));
  nand42aa1n04x5               g134(.a(\b[21] ), .b(\a[22] ), .o1(new_n230));
  norb02aa1n03x5               g135(.a(new_n230), .b(new_n229), .out0(new_n231));
  aoi112aa1n03x5               g136(.a(new_n226), .b(new_n231), .c(new_n224), .d(new_n228), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n231), .b(new_n226), .c(new_n224), .d(new_n227), .o1(new_n233));
  norb02aa1n02x7               g138(.a(new_n233), .b(new_n232), .out0(\s[22] ));
  nano23aa1n09x5               g139(.a(new_n226), .b(new_n229), .c(new_n230), .d(new_n227), .out0(new_n235));
  nand23aa1n04x5               g140(.a(new_n198), .b(new_n215), .c(new_n235), .o1(new_n236));
  tech160nm_fiao0012aa1n02p5x5 g141(.a(new_n229), .b(new_n226), .c(new_n230), .o(new_n237));
  aoi012aa1n02x5               g142(.a(new_n237), .b(new_n222), .c(new_n235), .o1(new_n238));
  aoai13aa1n04x5               g143(.a(new_n238), .b(new_n236), .c(new_n184), .d(new_n190), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  nanp02aa1n04x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  nor042aa1n03x5               g147(.a(\b[23] ), .b(\a[24] ), .o1(new_n243));
  nand42aa1n02x5               g148(.a(\b[23] ), .b(\a[24] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n241), .c(new_n239), .d(new_n242), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(new_n241), .b(new_n245), .c(new_n239), .d(new_n242), .o1(new_n247));
  norb02aa1n02x7               g152(.a(new_n246), .b(new_n247), .out0(\s[24] ));
  nano23aa1n09x5               g153(.a(new_n241), .b(new_n243), .c(new_n244), .d(new_n242), .out0(new_n249));
  nanb03aa1n03x5               g154(.a(new_n216), .b(new_n249), .c(new_n235), .out0(new_n250));
  aoi112aa1n02x5               g155(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(new_n249), .b(new_n237), .o1(new_n252));
  nona22aa1n02x4               g157(.a(new_n252), .b(new_n251), .c(new_n243), .out0(new_n253));
  nona23aa1n02x4               g158(.a(new_n244), .b(new_n242), .c(new_n241), .d(new_n243), .out0(new_n254));
  nano22aa1n03x7               g159(.a(new_n254), .b(new_n228), .c(new_n231), .out0(new_n255));
  aoi012aa1n02x5               g160(.a(new_n253), .b(new_n222), .c(new_n255), .o1(new_n256));
  aoai13aa1n04x5               g161(.a(new_n256), .b(new_n250), .c(new_n184), .d(new_n190), .o1(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  xorc02aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .out0(new_n261));
  aoi112aa1n02x5               g166(.a(new_n259), .b(new_n261), .c(new_n257), .d(new_n260), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n261), .b(new_n259), .c(new_n257), .d(new_n260), .o1(new_n263));
  norb02aa1n02x7               g168(.a(new_n263), .b(new_n262), .out0(\s[26] ));
  and002aa1n02x5               g169(.a(new_n261), .b(new_n260), .o(new_n265));
  nano22aa1n12x5               g170(.a(new_n236), .b(new_n265), .c(new_n249), .out0(new_n266));
  norp02aa1n02x5               g171(.a(\b[25] ), .b(\a[26] ), .o1(new_n267));
  aoi112aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n268));
  aoai13aa1n04x5               g173(.a(new_n265), .b(new_n253), .c(new_n222), .d(new_n255), .o1(new_n269));
  nona22aa1n12x5               g174(.a(new_n269), .b(new_n268), .c(new_n267), .out0(new_n270));
  aoi012aa1n06x5               g175(.a(new_n270), .b(new_n191), .c(new_n266), .o1(new_n271));
  nor042aa1n03x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  inv040aa1n03x5               g177(.a(new_n272), .o1(new_n273));
  nanp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n271), .b(new_n274), .c(new_n273), .out0(\s[27] ));
  xorc02aa1n12x5               g180(.a(\a[28] ), .b(\b[27] ), .out0(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n274), .b(new_n270), .c(new_n191), .d(new_n266), .o1(new_n278));
  tech160nm_fiaoi012aa1n03p5x5 g183(.a(new_n277), .b(new_n278), .c(new_n273), .o1(new_n279));
  nona22aa1n02x5               g184(.a(new_n278), .b(new_n276), .c(new_n272), .out0(new_n280));
  norb02aa1n03x4               g185(.a(new_n280), .b(new_n279), .out0(\s[28] ));
  nano22aa1n02x4               g186(.a(new_n277), .b(new_n273), .c(new_n274), .out0(new_n282));
  aoai13aa1n06x5               g187(.a(new_n282), .b(new_n270), .c(new_n191), .d(new_n266), .o1(new_n283));
  tech160nm_fioaoi03aa1n03p5x5 g188(.a(\a[28] ), .b(\b[27] ), .c(new_n273), .o1(new_n284));
  inv000aa1n03x5               g189(.a(new_n284), .o1(new_n285));
  tech160nm_fixorc02aa1n03p5x5 g190(.a(\a[29] ), .b(\b[28] ), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n286), .o1(new_n287));
  tech160nm_fiaoi012aa1n03p5x5 g192(.a(new_n287), .b(new_n283), .c(new_n285), .o1(new_n288));
  nona22aa1n02x5               g193(.a(new_n283), .b(new_n284), .c(new_n286), .out0(new_n289));
  norb02aa1n03x4               g194(.a(new_n289), .b(new_n288), .out0(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n115), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano32aa1n02x4               g196(.a(new_n287), .b(new_n276), .c(new_n274), .d(new_n273), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n270), .c(new_n191), .d(new_n266), .o1(new_n293));
  oao003aa1n09x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n294));
  xorc02aa1n06x5               g199(.a(\a[30] ), .b(\b[29] ), .out0(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  tech160nm_fiaoi012aa1n03p5x5 g201(.a(new_n296), .b(new_n293), .c(new_n294), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n294), .o1(new_n298));
  nona22aa1n02x5               g203(.a(new_n293), .b(new_n298), .c(new_n295), .out0(new_n299));
  norb02aa1n03x4               g204(.a(new_n299), .b(new_n297), .out0(\s[30] ));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  and003aa1n03x7               g207(.a(new_n282), .b(new_n295), .c(new_n286), .o(new_n303));
  aoai13aa1n06x5               g208(.a(new_n303), .b(new_n270), .c(new_n191), .d(new_n266), .o1(new_n304));
  oaoi03aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .o1(new_n305));
  nona22aa1n02x5               g210(.a(new_n304), .b(new_n305), .c(new_n302), .out0(new_n306));
  inv000aa1n02x5               g211(.a(new_n305), .o1(new_n307));
  tech160nm_fiaoi012aa1n03p5x5 g212(.a(new_n301), .b(new_n304), .c(new_n307), .o1(new_n308));
  norb02aa1n03x4               g213(.a(new_n306), .b(new_n308), .out0(\s[31] ));
  xnrb03aa1n02x5               g214(.a(new_n117), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g215(.a(\a[3] ), .b(\b[2] ), .c(new_n117), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g218(.a(new_n105), .b(new_n119), .c(new_n106), .o1(new_n314));
  xnrb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g220(.a(new_n100), .o1(new_n316));
  oaoi13aa1n02x5               g221(.a(new_n107), .b(new_n118), .c(new_n113), .d(new_n117), .o1(new_n317));
  norp03aa1n02x5               g222(.a(new_n317), .b(new_n121), .c(new_n103), .o1(new_n318));
  xnbna2aa1n03x5               g223(.a(new_n318), .b(new_n316), .c(new_n101), .out0(\s[7] ));
  oai013aa1n02x4               g224(.a(new_n102), .b(new_n317), .c(new_n103), .d(new_n121), .o1(new_n320));
  xnbna2aa1n03x5               g225(.a(new_n99), .b(new_n320), .c(new_n316), .out0(\s[8] ));
  xnrb03aa1n02x5               g226(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


