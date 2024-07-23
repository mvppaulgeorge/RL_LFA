// Benchmark "adder" written by ABC on Thu Jul 18 12:38:03 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n310, new_n312,
    new_n314, new_n316;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n03x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  inv000aa1d42x5               g004(.a(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[8] ), .o1(new_n101));
  nor002aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nand02aa1n03x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  nor022aa1n04x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nano23aa1n03x5               g010(.a(new_n102), .b(new_n104), .c(new_n105), .d(new_n103), .out0(new_n106));
  norp02aa1n02x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nand02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nor042aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  tech160nm_fiao0012aa1n02p5x5 g014(.a(new_n107), .b(new_n109), .c(new_n108), .o(new_n110));
  ao0012aa1n03x5               g015(.a(new_n102), .b(new_n104), .c(new_n103), .o(new_n111));
  aoi012aa1n06x5               g016(.a(new_n111), .b(new_n106), .c(new_n110), .o1(new_n112));
  nor042aa1n06x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  inv000aa1n02x5               g018(.a(new_n113), .o1(new_n114));
  oao003aa1n02x5               g019(.a(\a[4] ), .b(\b[3] ), .c(new_n114), .carry(new_n115));
  xorc02aa1n02x5               g020(.a(\a[4] ), .b(\b[3] ), .out0(new_n116));
  nanp02aa1n02x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  norb02aa1n02x5               g022(.a(new_n117), .b(new_n113), .out0(new_n118));
  and002aa1n02x7               g023(.a(\b[1] ), .b(\a[2] ), .o(new_n119));
  nand02aa1n02x5               g024(.a(\b[0] ), .b(\a[1] ), .o1(new_n120));
  norp02aa1n02x5               g025(.a(\b[1] ), .b(\a[2] ), .o1(new_n121));
  oab012aa1n06x5               g026(.a(new_n119), .b(new_n121), .c(new_n120), .out0(new_n122));
  nand03aa1n04x5               g027(.a(new_n122), .b(new_n116), .c(new_n118), .o1(new_n123));
  nanp02aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  nano23aa1n02x4               g029(.a(new_n107), .b(new_n109), .c(new_n124), .d(new_n108), .out0(new_n125));
  nanp02aa1n03x5               g030(.a(new_n125), .b(new_n106), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n112), .b(new_n126), .c(new_n123), .d(new_n115), .o1(new_n127));
  tech160nm_fioaoi03aa1n03p5x5 g032(.a(new_n100), .b(new_n101), .c(new_n127), .o1(new_n128));
  xnrc02aa1n02x5               g033(.a(new_n128), .b(new_n99), .out0(\s[10] ));
  tech160nm_fioaoi03aa1n03p5x5 g034(.a(\a[10] ), .b(\b[9] ), .c(new_n128), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  tech160nm_finand02aa1n03p5x5 g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  nor042aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n04x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n132), .c(new_n130), .d(new_n134), .o1(new_n138));
  nand42aa1n03x5               g043(.a(new_n130), .b(new_n134), .o1(new_n139));
  nona22aa1n02x4               g044(.a(new_n139), .b(new_n137), .c(new_n132), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(new_n140), .b(new_n138), .o1(\s[12] ));
  nona23aa1n02x4               g046(.a(new_n136), .b(new_n133), .c(new_n132), .d(new_n135), .out0(new_n142));
  aoai13aa1n02x5               g047(.a(new_n98), .b(new_n97), .c(new_n100), .d(new_n101), .o1(new_n143));
  ao0012aa1n03x5               g048(.a(new_n135), .b(new_n132), .c(new_n136), .o(new_n144));
  oabi12aa1n03x5               g049(.a(new_n144), .b(new_n142), .c(new_n143), .out0(new_n145));
  nano23aa1n06x5               g050(.a(new_n132), .b(new_n135), .c(new_n136), .d(new_n133), .out0(new_n146));
  xnrc02aa1n02x5               g051(.a(\b[8] ), .b(\a[9] ), .out0(new_n147));
  nanb03aa1d18x5               g052(.a(new_n147), .b(new_n146), .c(new_n99), .out0(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  xnrc02aa1n12x5               g054(.a(\b[12] ), .b(\a[13] ), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n145), .c(new_n127), .d(new_n149), .o1(new_n152));
  aoi112aa1n02x5               g057(.a(new_n145), .b(new_n151), .c(new_n127), .d(new_n149), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(\s[13] ));
  orn002aa1n02x5               g059(.a(\a[13] ), .b(\b[12] ), .o(new_n155));
  tech160nm_fixnrc02aa1n04x5   g060(.a(\b[13] ), .b(\a[14] ), .out0(new_n156));
  xobna2aa1n03x5               g061(.a(new_n156), .b(new_n152), .c(new_n155), .out0(\s[14] ));
  nor042aa1n03x5               g062(.a(new_n156), .b(new_n150), .o1(new_n158));
  oao003aa1n02x5               g063(.a(\a[14] ), .b(\b[13] ), .c(new_n155), .carry(new_n159));
  aobi12aa1n06x5               g064(.a(new_n159), .b(new_n145), .c(new_n158), .out0(new_n160));
  nona32aa1n03x5               g065(.a(new_n127), .b(new_n156), .c(new_n150), .d(new_n148), .out0(new_n161));
  xnrc02aa1n12x5               g066(.a(\b[14] ), .b(\a[15] ), .out0(new_n162));
  xobna2aa1n03x5               g067(.a(new_n162), .b(new_n161), .c(new_n160), .out0(\s[15] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  tech160nm_fiao0012aa1n02p5x5 g069(.a(new_n162), .b(new_n161), .c(new_n160), .o(new_n165));
  xnrc02aa1n06x5               g070(.a(\b[15] ), .b(\a[16] ), .out0(new_n166));
  oaib12aa1n02x5               g071(.a(new_n166), .b(new_n164), .c(new_n165), .out0(new_n167));
  nona22aa1n02x4               g072(.a(new_n165), .b(new_n166), .c(new_n164), .out0(new_n168));
  nanp02aa1n02x5               g073(.a(new_n167), .b(new_n168), .o1(\s[16] ));
  inv000aa1d42x5               g074(.a(\a[17] ), .o1(new_n170));
  nor042aa1n03x5               g075(.a(new_n166), .b(new_n162), .o1(new_n171));
  nano22aa1n09x5               g076(.a(new_n148), .b(new_n158), .c(new_n171), .out0(new_n172));
  nanp02aa1n02x5               g077(.a(new_n101), .b(new_n100), .o1(new_n173));
  oaoi03aa1n02x5               g078(.a(\a[10] ), .b(\b[9] ), .c(new_n173), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n158), .b(new_n144), .c(new_n146), .d(new_n174), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n171), .o1(new_n176));
  aoi012aa1n03x5               g081(.a(new_n176), .b(new_n175), .c(new_n159), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\a[16] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\b[15] ), .o1(new_n179));
  oaoi03aa1n12x5               g084(.a(new_n178), .b(new_n179), .c(new_n164), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  aoi112aa1n09x5               g086(.a(new_n177), .b(new_n181), .c(new_n127), .d(new_n172), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(new_n170), .out0(\s[17] ));
  oaoi03aa1n03x5               g088(.a(\a[17] ), .b(\b[16] ), .c(new_n182), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g090(.a(\a[18] ), .o1(new_n186));
  xroi22aa1d06x4               g091(.a(new_n170), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n187));
  inv000aa1n02x5               g092(.a(new_n187), .o1(new_n188));
  aoi112aa1n02x7               g093(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n189));
  aoib12aa1n06x5               g094(.a(new_n189), .b(new_n186), .c(\b[17] ), .out0(new_n190));
  tech160nm_fioai012aa1n05x5   g095(.a(new_n190), .b(new_n182), .c(new_n188), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g097(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nand22aa1n04x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  nor002aa1n06x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nand22aa1n06x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  aoai13aa1n03x5               g104(.a(new_n199), .b(new_n194), .c(new_n191), .d(new_n196), .o1(new_n200));
  nanp02aa1n06x5               g105(.a(new_n127), .b(new_n172), .o1(new_n201));
  oai112aa1n06x5               g106(.a(new_n201), .b(new_n180), .c(new_n176), .d(new_n160), .o1(new_n202));
  nor002aa1n02x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  aob012aa1n02x5               g108(.a(new_n203), .b(\b[17] ), .c(\a[18] ), .out0(new_n204));
  oaib12aa1n09x5               g109(.a(new_n204), .b(\b[17] ), .c(new_n186), .out0(new_n205));
  aoai13aa1n02x5               g110(.a(new_n196), .b(new_n205), .c(new_n202), .d(new_n187), .o1(new_n206));
  nona22aa1n02x4               g111(.a(new_n206), .b(new_n199), .c(new_n194), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n200), .b(new_n207), .o1(\s[20] ));
  nona23aa1n09x5               g113(.a(new_n198), .b(new_n195), .c(new_n194), .d(new_n197), .out0(new_n209));
  ao0012aa1n12x5               g114(.a(new_n197), .b(new_n194), .c(new_n198), .o(new_n210));
  oabi12aa1n18x5               g115(.a(new_n210), .b(new_n209), .c(new_n190), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  norb02aa1n06x5               g117(.a(new_n187), .b(new_n209), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  tech160nm_fioai012aa1n05x5   g119(.a(new_n212), .b(new_n182), .c(new_n214), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xnrc02aa1n12x5               g122(.a(\b[20] ), .b(\a[21] ), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  tech160nm_fixnrc02aa1n05x5   g124(.a(\b[21] ), .b(\a[22] ), .out0(new_n220));
  aoai13aa1n03x5               g125(.a(new_n220), .b(new_n217), .c(new_n215), .d(new_n219), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n219), .b(new_n211), .c(new_n202), .d(new_n213), .o1(new_n222));
  nona22aa1n02x4               g127(.a(new_n222), .b(new_n220), .c(new_n217), .out0(new_n223));
  nanp02aa1n02x5               g128(.a(new_n221), .b(new_n223), .o1(\s[22] ));
  nor042aa1n06x5               g129(.a(new_n220), .b(new_n218), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\a[22] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\b[21] ), .o1(new_n227));
  oaoi03aa1n09x5               g132(.a(new_n226), .b(new_n227), .c(new_n217), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoi012aa1d18x5               g134(.a(new_n229), .b(new_n211), .c(new_n225), .o1(new_n230));
  nano23aa1n06x5               g135(.a(new_n194), .b(new_n197), .c(new_n198), .d(new_n195), .out0(new_n231));
  nano22aa1d15x5               g136(.a(new_n188), .b(new_n225), .c(new_n231), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  tech160nm_fioai012aa1n05x5   g138(.a(new_n230), .b(new_n182), .c(new_n233), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  xorc02aa1n12x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  xnrc02aa1n02x5               g142(.a(\b[23] ), .b(\a[24] ), .out0(new_n238));
  aoai13aa1n03x5               g143(.a(new_n238), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n230), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n237), .b(new_n240), .c(new_n202), .d(new_n232), .o1(new_n241));
  nona22aa1n02x4               g146(.a(new_n241), .b(new_n238), .c(new_n236), .out0(new_n242));
  nanp02aa1n03x5               g147(.a(new_n239), .b(new_n242), .o1(\s[24] ));
  norb02aa1n02x5               g148(.a(new_n237), .b(new_n238), .out0(new_n244));
  inv030aa1n02x5               g149(.a(new_n244), .o1(new_n245));
  nano32aa1n03x7               g150(.a(new_n245), .b(new_n187), .c(new_n225), .d(new_n231), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  aoai13aa1n09x5               g152(.a(new_n225), .b(new_n210), .c(new_n231), .d(new_n205), .o1(new_n248));
  orn002aa1n02x5               g153(.a(\a[23] ), .b(\b[22] ), .o(new_n249));
  oao003aa1n02x5               g154(.a(\a[24] ), .b(\b[23] ), .c(new_n249), .carry(new_n250));
  aoai13aa1n12x5               g155(.a(new_n250), .b(new_n245), .c(new_n248), .d(new_n228), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  tech160nm_fioai012aa1n05x5   g157(.a(new_n252), .b(new_n182), .c(new_n247), .o1(new_n253));
  xorb03aa1n02x5               g158(.a(new_n253), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  xorc02aa1n06x5               g160(.a(\a[25] ), .b(\b[24] ), .out0(new_n256));
  xnrc02aa1n12x5               g161(.a(\b[25] ), .b(\a[26] ), .out0(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n255), .c(new_n253), .d(new_n256), .o1(new_n258));
  aoai13aa1n03x5               g163(.a(new_n256), .b(new_n251), .c(new_n202), .d(new_n246), .o1(new_n259));
  nona22aa1n02x4               g164(.a(new_n259), .b(new_n257), .c(new_n255), .out0(new_n260));
  nanp02aa1n02x5               g165(.a(new_n258), .b(new_n260), .o1(\s[26] ));
  norb02aa1n02x5               g166(.a(new_n256), .b(new_n257), .out0(new_n262));
  inv000aa1d42x5               g167(.a(\a[26] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(\b[25] ), .o1(new_n264));
  oaoi03aa1n02x5               g169(.a(new_n263), .b(new_n264), .c(new_n255), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  tech160nm_fiaoi012aa1n05x5   g171(.a(new_n266), .b(new_n251), .c(new_n262), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n262), .o1(new_n268));
  nona22aa1n12x5               g173(.a(new_n232), .b(new_n268), .c(new_n245), .out0(new_n269));
  oai012aa1n06x5               g174(.a(new_n267), .b(new_n182), .c(new_n269), .o1(new_n270));
  xorb03aa1n02x5               g175(.a(new_n270), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  xorc02aa1n02x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[27] ), .b(\a[28] ), .out0(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n272), .c(new_n270), .d(new_n273), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n244), .b(new_n229), .c(new_n211), .d(new_n225), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n265), .b(new_n268), .c(new_n276), .d(new_n250), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n269), .o1(new_n278));
  aoai13aa1n04x5               g183(.a(new_n273), .b(new_n277), .c(new_n202), .d(new_n278), .o1(new_n279));
  nona22aa1n02x5               g184(.a(new_n279), .b(new_n274), .c(new_n272), .out0(new_n280));
  nanp02aa1n03x5               g185(.a(new_n275), .b(new_n280), .o1(\s[28] ));
  norb02aa1n02x5               g186(.a(new_n273), .b(new_n274), .out0(new_n282));
  aoai13aa1n02x5               g187(.a(new_n282), .b(new_n277), .c(new_n202), .d(new_n278), .o1(new_n283));
  aob012aa1n02x5               g188(.a(new_n272), .b(\b[27] ), .c(\a[28] ), .out0(new_n284));
  oa0012aa1n12x5               g189(.a(new_n284), .b(\b[27] ), .c(\a[28] ), .o(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[28] ), .b(\a[29] ), .out0(new_n287));
  nona22aa1n03x5               g192(.a(new_n283), .b(new_n286), .c(new_n287), .out0(new_n288));
  aoai13aa1n03x5               g193(.a(new_n287), .b(new_n286), .c(new_n270), .d(new_n282), .o1(new_n289));
  nanp02aa1n03x5               g194(.a(new_n289), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n120), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g196(.a(new_n273), .b(new_n287), .c(new_n274), .out0(new_n292));
  oaoi03aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .o1(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n293), .c(new_n270), .d(new_n292), .o1(new_n295));
  aoai13aa1n02x7               g200(.a(new_n292), .b(new_n277), .c(new_n202), .d(new_n278), .o1(new_n296));
  nona22aa1n02x5               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  nanp02aa1n03x5               g202(.a(new_n295), .b(new_n297), .o1(\s[30] ));
  nanb02aa1n02x5               g203(.a(new_n294), .b(new_n293), .out0(new_n299));
  oai012aa1n02x5               g204(.a(new_n299), .b(\b[29] ), .c(\a[30] ), .o1(new_n300));
  norb02aa1n02x5               g205(.a(new_n292), .b(new_n294), .out0(new_n301));
  aoai13aa1n02x7               g206(.a(new_n301), .b(new_n277), .c(new_n202), .d(new_n278), .o1(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nona22aa1n02x5               g208(.a(new_n302), .b(new_n303), .c(new_n300), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n303), .b(new_n300), .c(new_n270), .d(new_n301), .o1(new_n305));
  nanp02aa1n03x5               g210(.a(new_n305), .b(new_n304), .o1(\s[31] ));
  xobna2aa1n03x5               g211(.a(new_n122), .b(new_n117), .c(new_n114), .out0(\s[3] ));
  oai012aa1n02x5               g212(.a(new_n117), .b(new_n122), .c(new_n113), .o1(new_n308));
  xnrb03aa1n02x5               g213(.a(new_n308), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  nanp02aa1n02x5               g214(.a(new_n123), .b(new_n115), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g216(.a(new_n109), .b(new_n310), .c(new_n124), .o1(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g218(.a(new_n110), .b(new_n310), .c(new_n125), .o(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g220(.a(new_n104), .b(new_n314), .c(new_n105), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g222(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


