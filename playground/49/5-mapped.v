// Benchmark "adder" written by ABC on Thu Jul 18 14:33:48 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n310, new_n312, new_n314;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_finand02aa1n03p5x5 g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1d08x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nor042aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  oai012aa1n12x5               g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  tech160nm_fixnrc02aa1n04x5   g005(.a(\b[3] ), .b(\a[4] ), .out0(new_n101));
  nor002aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand42aa1n03x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb02aa1n09x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[3] ), .o1(new_n105));
  nanb02aa1n12x5               g010(.a(\b[2] ), .b(new_n105), .out0(new_n106));
  oao003aa1n03x5               g011(.a(\a[4] ), .b(\b[3] ), .c(new_n106), .carry(new_n107));
  oai013aa1d12x5               g012(.a(new_n107), .b(new_n101), .c(new_n100), .d(new_n104), .o1(new_n108));
  xnrc02aa1n12x5               g013(.a(\b[7] ), .b(\a[8] ), .out0(new_n109));
  xnrc02aa1n12x5               g014(.a(\b[6] ), .b(\a[7] ), .out0(new_n110));
  norp02aa1n06x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nand42aa1n20x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  norp02aa1n24x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand42aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  nor043aa1n06x5               g020(.a(new_n115), .b(new_n110), .c(new_n109), .o1(new_n116));
  aoi012aa1n06x5               g021(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n117));
  inv000aa1d42x5               g022(.a(\a[8] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[7] ), .o1(new_n119));
  norp02aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n121));
  oai013aa1n06x5               g026(.a(new_n121), .b(new_n110), .c(new_n109), .d(new_n117), .o1(new_n122));
  aoi012aa1n02x5               g027(.a(new_n122), .b(new_n108), .c(new_n116), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .c(new_n123), .o1(new_n124));
  xorb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n04x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nor042aa1n04x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand02aa1n03x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  tech160nm_fiaoi012aa1n05x5   g033(.a(new_n127), .b(new_n126), .c(new_n128), .o1(new_n129));
  nand42aa1n02x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nano23aa1n06x5               g035(.a(new_n126), .b(new_n127), .c(new_n128), .d(new_n130), .out0(new_n131));
  aoai13aa1n02x5               g036(.a(new_n131), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(new_n132), .b(new_n129), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g039(.a(\a[12] ), .o1(new_n135));
  nor022aa1n12x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand02aa1n06x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  aoi012aa1n02x5               g042(.a(new_n136), .b(new_n133), .c(new_n137), .o1(new_n138));
  xorb03aa1n02x5               g043(.a(new_n138), .b(\b[11] ), .c(new_n135), .out0(\s[12] ));
  nor022aa1n16x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand22aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nona23aa1d18x5               g046(.a(new_n141), .b(new_n137), .c(new_n136), .d(new_n140), .out0(new_n142));
  tech160nm_fiaoi012aa1n05x5   g047(.a(new_n140), .b(new_n136), .c(new_n141), .o1(new_n143));
  oai012aa1n18x5               g048(.a(new_n143), .b(new_n142), .c(new_n129), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n131), .b(new_n142), .out0(new_n146));
  aoai13aa1n06x5               g051(.a(new_n146), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n147), .b(new_n145), .o1(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g054(.a(\a[14] ), .o1(new_n150));
  nor042aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nand42aa1n20x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n151), .b(new_n148), .c(new_n152), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[13] ), .c(new_n150), .out0(\s[14] ));
  nor042aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nand42aa1n08x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nona23aa1n02x4               g061(.a(new_n156), .b(new_n152), .c(new_n151), .d(new_n155), .out0(new_n157));
  nano23aa1n03x7               g062(.a(new_n151), .b(new_n155), .c(new_n156), .d(new_n152), .out0(new_n158));
  aoi012aa1n02x5               g063(.a(new_n155), .b(new_n151), .c(new_n156), .o1(new_n159));
  aobi12aa1n02x5               g064(.a(new_n159), .b(new_n144), .c(new_n158), .out0(new_n160));
  tech160nm_fioai012aa1n03p5x5 g065(.a(new_n160), .b(new_n147), .c(new_n157), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nand42aa1n16x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nor042aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nand42aa1n06x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nanb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n163), .c(new_n161), .d(new_n164), .o1(new_n168));
  aoi112aa1n03x5               g073(.a(new_n163), .b(new_n167), .c(new_n161), .d(new_n164), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(\s[16] ));
  nano23aa1n06x5               g075(.a(new_n136), .b(new_n140), .c(new_n141), .d(new_n137), .out0(new_n171));
  nano23aa1n03x7               g076(.a(new_n163), .b(new_n165), .c(new_n166), .d(new_n164), .out0(new_n172));
  nand22aa1n03x5               g077(.a(new_n172), .b(new_n158), .o1(new_n173));
  nano22aa1n03x7               g078(.a(new_n173), .b(new_n131), .c(new_n171), .out0(new_n174));
  aoai13aa1n12x5               g079(.a(new_n174), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n175));
  aoi012aa1n02x5               g080(.a(new_n165), .b(new_n163), .c(new_n166), .o1(new_n176));
  oaib12aa1n02x5               g081(.a(new_n176), .b(new_n159), .c(new_n172), .out0(new_n177));
  aoib12aa1n12x5               g082(.a(new_n177), .b(new_n144), .c(new_n173), .out0(new_n178));
  xorc02aa1n12x5               g083(.a(\a[17] ), .b(\b[16] ), .out0(new_n179));
  xnbna2aa1n03x5               g084(.a(new_n179), .b(new_n175), .c(new_n178), .out0(\s[17] ));
  nanp02aa1n12x5               g085(.a(new_n175), .b(new_n178), .o1(new_n181));
  nor042aa1n04x5               g086(.a(\b[16] ), .b(\a[17] ), .o1(new_n182));
  aoi012aa1n06x5               g087(.a(new_n182), .b(new_n181), .c(new_n179), .o1(new_n183));
  inv040aa1d32x5               g088(.a(\a[18] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[17] ), .o1(new_n185));
  nanp02aa1n04x5               g090(.a(new_n185), .b(new_n184), .o1(new_n186));
  nand02aa1d06x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  xnbna2aa1n03x5               g092(.a(new_n183), .b(new_n187), .c(new_n186), .out0(\s[18] ));
  oaoi03aa1n02x5               g093(.a(new_n184), .b(new_n185), .c(new_n182), .o1(new_n189));
  inv000aa1n09x5               g094(.a(new_n179), .o1(new_n190));
  nano22aa1d15x5               g095(.a(new_n190), .b(new_n186), .c(new_n187), .out0(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  aoai13aa1n06x5               g097(.a(new_n189), .b(new_n192), .c(new_n175), .d(new_n178), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nand42aa1n04x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nor002aa1n16x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nanp02aa1n24x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  norb02aa1n12x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  aoai13aa1n03x5               g106(.a(new_n201), .b(new_n196), .c(new_n193), .d(new_n197), .o1(new_n202));
  norb02aa1n06x4               g107(.a(new_n197), .b(new_n196), .out0(new_n203));
  nand02aa1n03x5               g108(.a(new_n193), .b(new_n203), .o1(new_n204));
  nona22aa1n02x5               g109(.a(new_n204), .b(new_n201), .c(new_n196), .out0(new_n205));
  nanp02aa1n03x5               g110(.a(new_n205), .b(new_n202), .o1(\s[20] ));
  aob012aa1n02x5               g111(.a(new_n186), .b(new_n182), .c(new_n187), .out0(new_n207));
  nano23aa1d12x5               g112(.a(new_n196), .b(new_n198), .c(new_n199), .d(new_n197), .out0(new_n208));
  aoi012aa1d18x5               g113(.a(new_n198), .b(new_n196), .c(new_n199), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  aoi012aa1n02x5               g115(.a(new_n210), .b(new_n208), .c(new_n207), .o1(new_n211));
  nanp02aa1n02x5               g116(.a(new_n191), .b(new_n208), .o1(new_n212));
  aoai13aa1n06x5               g117(.a(new_n211), .b(new_n212), .c(new_n175), .d(new_n178), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  nand42aa1d28x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  nor022aa1n16x5               g122(.a(\b[21] ), .b(\a[22] ), .o1(new_n218));
  nand02aa1d28x5               g123(.a(\b[21] ), .b(\a[22] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n215), .c(new_n213), .d(new_n217), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(new_n213), .b(new_n217), .o1(new_n223));
  nona22aa1n02x4               g128(.a(new_n223), .b(new_n221), .c(new_n215), .out0(new_n224));
  nanp02aa1n02x5               g129(.a(new_n224), .b(new_n222), .o1(\s[22] ));
  nano23aa1d15x5               g130(.a(new_n215), .b(new_n218), .c(new_n219), .d(new_n216), .out0(new_n226));
  nanp03aa1n02x5               g131(.a(new_n191), .b(new_n208), .c(new_n226), .o1(new_n227));
  nano22aa1n02x4               g132(.a(new_n189), .b(new_n203), .c(new_n200), .out0(new_n228));
  ao0012aa1n03x7               g133(.a(new_n218), .b(new_n215), .c(new_n219), .o(new_n229));
  oaoi13aa1n02x5               g134(.a(new_n229), .b(new_n226), .c(new_n228), .d(new_n210), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n227), .c(new_n175), .d(new_n178), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  nanp02aa1n12x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  norb02aa1n02x5               g139(.a(new_n234), .b(new_n233), .out0(new_n235));
  nor042aa1n04x5               g140(.a(\b[23] ), .b(\a[24] ), .o1(new_n236));
  nanp02aa1n12x5               g141(.a(\b[23] ), .b(\a[24] ), .o1(new_n237));
  nanb02aa1n02x5               g142(.a(new_n236), .b(new_n237), .out0(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n233), .c(new_n231), .d(new_n235), .o1(new_n239));
  nanp02aa1n02x5               g144(.a(new_n231), .b(new_n235), .o1(new_n240));
  nona22aa1n02x4               g145(.a(new_n240), .b(new_n238), .c(new_n233), .out0(new_n241));
  nanp02aa1n02x5               g146(.a(new_n241), .b(new_n239), .o1(\s[24] ));
  nanp03aa1n06x5               g147(.a(new_n207), .b(new_n203), .c(new_n200), .o1(new_n243));
  nano23aa1d15x5               g148(.a(new_n233), .b(new_n236), .c(new_n237), .d(new_n234), .out0(new_n244));
  nand22aa1n09x5               g149(.a(new_n244), .b(new_n226), .o1(new_n245));
  ao0012aa1n03x7               g150(.a(new_n236), .b(new_n233), .c(new_n237), .o(new_n246));
  aoi012aa1n12x5               g151(.a(new_n246), .b(new_n244), .c(new_n229), .o1(new_n247));
  aoai13aa1n12x5               g152(.a(new_n247), .b(new_n245), .c(new_n243), .d(new_n209), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  nanb03aa1n02x5               g154(.a(new_n245), .b(new_n191), .c(new_n208), .out0(new_n250));
  aoai13aa1n04x5               g155(.a(new_n249), .b(new_n250), .c(new_n175), .d(new_n178), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n06x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  xnrc02aa1n02x5               g159(.a(\b[25] ), .b(\a[26] ), .out0(new_n255));
  aoai13aa1n03x5               g160(.a(new_n255), .b(new_n253), .c(new_n251), .d(new_n254), .o1(new_n256));
  nand22aa1n02x5               g161(.a(new_n251), .b(new_n254), .o1(new_n257));
  nona22aa1n03x5               g162(.a(new_n257), .b(new_n255), .c(new_n253), .out0(new_n258));
  nanp02aa1n03x5               g163(.a(new_n258), .b(new_n256), .o1(\s[26] ));
  inv000aa1d42x5               g164(.a(new_n245), .o1(new_n260));
  norb02aa1n02x7               g165(.a(new_n254), .b(new_n255), .out0(new_n261));
  nanb03aa1n02x5               g166(.a(new_n212), .b(new_n261), .c(new_n260), .out0(new_n262));
  inv000aa1d42x5               g167(.a(\a[26] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(\b[25] ), .o1(new_n264));
  oaoi03aa1n02x5               g169(.a(new_n263), .b(new_n264), .c(new_n253), .o1(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  tech160nm_fiaoi012aa1n05x5   g171(.a(new_n266), .b(new_n248), .c(new_n261), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n262), .c(new_n175), .d(new_n178), .o1(new_n268));
  xorb03aa1n03x5               g173(.a(new_n268), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n03x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  aoai13aa1n03x5               g177(.a(new_n272), .b(new_n270), .c(new_n268), .d(new_n271), .o1(new_n273));
  nano22aa1n06x5               g178(.a(new_n212), .b(new_n260), .c(new_n261), .out0(new_n274));
  nanp02aa1n09x5               g179(.a(new_n248), .b(new_n261), .o1(new_n275));
  nanp02aa1n06x5               g180(.a(new_n275), .b(new_n265), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n271), .b(new_n276), .c(new_n181), .d(new_n274), .o1(new_n277));
  nona22aa1n03x5               g182(.a(new_n277), .b(new_n272), .c(new_n270), .out0(new_n278));
  nanp02aa1n03x5               g183(.a(new_n273), .b(new_n278), .o1(\s[28] ));
  inv000aa1d42x5               g184(.a(\a[28] ), .o1(new_n280));
  inv000aa1d42x5               g185(.a(\b[27] ), .o1(new_n281));
  oaoi03aa1n09x5               g186(.a(new_n280), .b(new_n281), .c(new_n270), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  norb02aa1n02x5               g188(.a(new_n271), .b(new_n272), .out0(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n276), .c(new_n181), .d(new_n274), .o1(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  nona22aa1n03x5               g191(.a(new_n285), .b(new_n286), .c(new_n283), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n286), .b(new_n283), .c(new_n268), .d(new_n284), .o1(new_n288));
  nanp02aa1n03x5               g193(.a(new_n288), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  tech160nm_fioaoi03aa1n03p5x5 g195(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .o1(new_n291));
  norb03aa1n02x5               g196(.a(new_n271), .b(new_n286), .c(new_n272), .out0(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n291), .c(new_n268), .d(new_n292), .o1(new_n294));
  aoai13aa1n06x5               g199(.a(new_n292), .b(new_n276), .c(new_n181), .d(new_n274), .o1(new_n295));
  nona22aa1n06x5               g200(.a(new_n295), .b(new_n293), .c(new_n291), .out0(new_n296));
  nanp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(\s[30] ));
  norb02aa1n02x5               g202(.a(new_n292), .b(new_n293), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n276), .c(new_n181), .d(new_n274), .o1(new_n299));
  nanb02aa1n02x5               g204(.a(new_n293), .b(new_n291), .out0(new_n300));
  tech160nm_fioai012aa1n03p5x5 g205(.a(new_n300), .b(\b[29] ), .c(\a[30] ), .o1(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nona22aa1n03x5               g207(.a(new_n299), .b(new_n301), .c(new_n302), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n302), .b(new_n301), .c(new_n268), .d(new_n298), .o1(new_n304));
  nanp02aa1n03x5               g209(.a(new_n304), .b(new_n303), .o1(\s[31] ));
  xnbna2aa1n03x5               g210(.a(new_n100), .b(new_n103), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g211(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g213(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g214(.a(new_n113), .b(new_n108), .c(new_n114), .o1(new_n310));
  xnrb03aa1n02x5               g215(.a(new_n310), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g216(.a(new_n117), .b(new_n115), .c(new_n108), .out0(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n02x5               g218(.a(new_n120), .b(new_n312), .c(new_n110), .out0(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[7] ), .c(new_n118), .out0(\s[8] ));
  xnrb03aa1n02x5               g220(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


