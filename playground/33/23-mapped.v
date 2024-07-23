// Benchmark "adder" written by ABC on Thu Jul 18 05:03:46 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n317,
    new_n318, new_n321, new_n323, new_n325;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[3] ), .o1(new_n97));
  inv040aa1d28x5               g002(.a(\b[2] ), .o1(new_n98));
  nand42aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nand42aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nand22aa1n06x5               g005(.a(new_n99), .b(new_n100), .o1(new_n101));
  orn002aa1n24x5               g006(.a(\a[2] ), .b(\b[1] ), .o(new_n102));
  nanp02aa1n06x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  aob012aa1n12x5               g008(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[4] ), .o1(new_n105));
  aboi22aa1n09x5               g010(.a(\b[3] ), .b(new_n105), .c(new_n97), .d(new_n98), .out0(new_n106));
  aoai13aa1n12x5               g011(.a(new_n106), .b(new_n101), .c(new_n104), .d(new_n102), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\a[6] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\b[5] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(new_n110), .b(new_n109), .o1(new_n111));
  tech160nm_finand02aa1n03p5x5 g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nor002aa1n16x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanb02aa1n03x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  nano32aa1n03x7               g020(.a(new_n115), .b(new_n111), .c(new_n112), .d(new_n108), .out0(new_n116));
  nand02aa1d04x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nor042aa1d18x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nor042aa1d18x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nand22aa1n04x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nona23aa1n09x5               g025(.a(new_n117), .b(new_n120), .c(new_n119), .d(new_n118), .out0(new_n121));
  inv040aa1n02x5               g026(.a(new_n121), .o1(new_n122));
  tech160nm_fioaoi03aa1n03p5x5 g027(.a(new_n109), .b(new_n110), .c(new_n113), .o1(new_n123));
  tech160nm_fiaoi012aa1n03p5x5 g028(.a(new_n118), .b(new_n119), .c(new_n117), .o1(new_n124));
  oaih12aa1n02x5               g029(.a(new_n124), .b(new_n121), .c(new_n123), .o1(new_n125));
  aoi013aa1n03x5               g030(.a(new_n125), .b(new_n107), .c(new_n122), .d(new_n116), .o1(new_n126));
  tech160nm_fioaoi03aa1n03p5x5 g031(.a(\a[9] ), .b(\b[8] ), .c(new_n126), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand22aa1n12x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand02aa1d08x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norp02aa1n04x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  oaoi13aa1n02x5               g038(.a(new_n133), .b(new_n130), .c(new_n127), .d(new_n129), .o1(new_n134));
  nor042aa1n04x5               g039(.a(new_n127), .b(new_n129), .o1(new_n135));
  nano22aa1n03x7               g040(.a(new_n135), .b(new_n130), .c(new_n133), .out0(new_n136));
  norp02aa1n02x5               g041(.a(new_n134), .b(new_n136), .o1(\s[11] ));
  xnrc02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .out0(new_n138));
  tech160nm_fioai012aa1n04x5   g043(.a(new_n138), .b(new_n136), .c(new_n132), .o1(new_n139));
  inv040aa1n03x5               g044(.a(new_n136), .o1(new_n140));
  nona22aa1n02x4               g045(.a(new_n140), .b(new_n138), .c(new_n132), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n141), .b(new_n139), .o1(\s[12] ));
  nanp03aa1d12x5               g047(.a(new_n107), .b(new_n122), .c(new_n116), .o1(new_n143));
  inv000aa1n03x5               g048(.a(new_n125), .o1(new_n144));
  nano23aa1n06x5               g049(.a(new_n132), .b(new_n129), .c(new_n130), .d(new_n131), .out0(new_n145));
  xnrc02aa1n03x5               g050(.a(\b[8] ), .b(\a[9] ), .out0(new_n146));
  nona22aa1n02x4               g051(.a(new_n145), .b(new_n146), .c(new_n138), .out0(new_n147));
  oai022aa1d18x5               g052(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n148));
  nand43aa1n03x5               g053(.a(new_n148), .b(new_n130), .c(new_n131), .o1(new_n149));
  oai122aa1n02x5               g054(.a(new_n149), .b(\a[12] ), .c(\b[11] ), .d(\a[11] ), .e(\b[10] ), .o1(new_n150));
  aob012aa1n02x5               g055(.a(new_n150), .b(\b[11] ), .c(\a[12] ), .out0(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n147), .c(new_n144), .d(new_n143), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nand02aa1d28x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  norb02aa1n06x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  nor042aa1d18x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand02aa1d06x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n157), .b(new_n152), .c(new_n158), .o1(new_n159));
  xnrc02aa1n02x5               g064(.a(new_n159), .b(new_n156), .out0(\s[14] ));
  nano23aa1n02x4               g065(.a(new_n157), .b(new_n154), .c(new_n155), .d(new_n158), .out0(new_n161));
  aoi012aa1n02x5               g066(.a(new_n154), .b(new_n157), .c(new_n155), .o1(new_n162));
  aobi12aa1n06x5               g067(.a(new_n162), .b(new_n152), .c(new_n161), .out0(new_n163));
  norp02aa1n12x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  inv030aa1n02x5               g069(.a(new_n164), .o1(new_n165));
  nand02aa1n08x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  xnbna2aa1n03x5               g071(.a(new_n163), .b(new_n166), .c(new_n165), .out0(\s[15] ));
  nor002aa1d32x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  nand02aa1d16x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  oaoi03aa1n02x5               g076(.a(\a[15] ), .b(\b[14] ), .c(new_n163), .o1(new_n172));
  oaib12aa1n02x5               g077(.a(new_n172), .b(new_n171), .c(new_n169), .out0(new_n173));
  nanb02aa1n02x5               g078(.a(new_n164), .b(new_n166), .out0(new_n174));
  norb02aa1n02x5               g079(.a(new_n170), .b(new_n168), .out0(new_n175));
  oai112aa1n02x5               g080(.a(new_n175), .b(new_n165), .c(new_n163), .d(new_n174), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(new_n173), .b(new_n176), .o1(\s[16] ));
  norb02aa1n03x5               g082(.a(new_n158), .b(new_n157), .out0(new_n178));
  nona23aa1n09x5               g083(.a(new_n170), .b(new_n166), .c(new_n164), .d(new_n168), .out0(new_n179));
  nano22aa1n03x7               g084(.a(new_n179), .b(new_n178), .c(new_n156), .out0(new_n180));
  nona23aa1n08x5               g085(.a(new_n180), .b(new_n145), .c(new_n138), .d(new_n146), .out0(new_n181));
  aoi022aa1n02x5               g086(.a(\b[15] ), .b(\a[16] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n182));
  nano32aa1n02x5               g087(.a(new_n179), .b(new_n156), .c(new_n178), .d(new_n182), .out0(new_n183));
  aoai13aa1n04x5               g088(.a(new_n166), .b(new_n154), .c(new_n157), .d(new_n155), .o1(new_n184));
  aoai13aa1n03x5               g089(.a(new_n169), .b(new_n171), .c(new_n184), .d(new_n165), .o1(new_n185));
  aoi012aa1n06x5               g090(.a(new_n185), .b(new_n183), .c(new_n150), .o1(new_n186));
  aoai13aa1n12x5               g091(.a(new_n186), .b(new_n181), .c(new_n144), .d(new_n143), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g093(.a(\a[18] ), .o1(new_n189));
  inv040aa1d32x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d06x4               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  oai022aa1n02x5               g099(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n195));
  oaib12aa1n06x5               g100(.a(new_n195), .b(new_n189), .c(\b[17] ), .out0(new_n196));
  aob012aa1n03x5               g101(.a(new_n196), .b(new_n187), .c(new_n194), .out0(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nor042aa1n04x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand02aa1n03x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  aoai13aa1n03x5               g110(.a(new_n205), .b(new_n200), .c(new_n197), .d(new_n202), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(new_n191), .b(new_n190), .o1(new_n207));
  oaoi03aa1n12x5               g112(.a(\a[18] ), .b(\b[17] ), .c(new_n207), .o1(new_n208));
  aoai13aa1n02x5               g113(.a(new_n202), .b(new_n208), .c(new_n187), .d(new_n194), .o1(new_n209));
  nona22aa1n02x4               g114(.a(new_n209), .b(new_n205), .c(new_n200), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n206), .b(new_n210), .o1(\s[20] ));
  nano23aa1n06x5               g116(.a(new_n200), .b(new_n203), .c(new_n204), .d(new_n201), .out0(new_n212));
  nand02aa1d04x5               g117(.a(new_n194), .b(new_n212), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  nona23aa1n03x5               g119(.a(new_n204), .b(new_n201), .c(new_n200), .d(new_n203), .out0(new_n215));
  tech160nm_fiaoi012aa1n04x5   g120(.a(new_n203), .b(new_n200), .c(new_n204), .o1(new_n216));
  oai012aa1n06x5               g121(.a(new_n216), .b(new_n215), .c(new_n196), .o1(new_n217));
  ao0012aa1n03x7               g122(.a(new_n217), .b(new_n187), .c(new_n214), .o(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xnrc02aa1n12x5               g125(.a(\b[20] ), .b(\a[21] ), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  tech160nm_fixnrc02aa1n05x5   g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  aoai13aa1n03x5               g128(.a(new_n223), .b(new_n220), .c(new_n218), .d(new_n222), .o1(new_n224));
  aoai13aa1n02x5               g129(.a(new_n222), .b(new_n217), .c(new_n187), .d(new_n214), .o1(new_n225));
  nona22aa1n02x4               g130(.a(new_n225), .b(new_n223), .c(new_n220), .out0(new_n226));
  nanp02aa1n03x5               g131(.a(new_n224), .b(new_n226), .o1(\s[22] ));
  nor042aa1d18x5               g132(.a(new_n223), .b(new_n221), .o1(new_n228));
  nand23aa1n03x5               g133(.a(new_n194), .b(new_n228), .c(new_n212), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\a[22] ), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\b[21] ), .o1(new_n231));
  oaoi03aa1n12x5               g136(.a(new_n230), .b(new_n231), .c(new_n220), .o1(new_n232));
  inv030aa1n02x5               g137(.a(new_n232), .o1(new_n233));
  aoi012aa1n12x5               g138(.a(new_n233), .b(new_n217), .c(new_n228), .o1(new_n234));
  oaib12aa1n06x5               g139(.a(new_n234), .b(new_n229), .c(new_n187), .out0(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  xorc02aa1n12x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[23] ), .b(\a[24] ), .out0(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n240));
  oaoi13aa1n04x5               g145(.a(new_n229), .b(new_n186), .c(new_n126), .d(new_n181), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n234), .o1(new_n242));
  oai012aa1n02x5               g147(.a(new_n238), .b(new_n241), .c(new_n242), .o1(new_n243));
  nona22aa1n02x4               g148(.a(new_n243), .b(new_n239), .c(new_n237), .out0(new_n244));
  nanp02aa1n03x5               g149(.a(new_n240), .b(new_n244), .o1(\s[24] ));
  inv000aa1n02x5               g150(.a(new_n216), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n228), .b(new_n246), .c(new_n212), .d(new_n208), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n238), .b(new_n239), .out0(new_n248));
  inv040aa1n02x5               g153(.a(new_n248), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n250));
  oab012aa1n02x4               g155(.a(new_n250), .b(\a[24] ), .c(\b[23] ), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n249), .c(new_n247), .d(new_n232), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n228), .o1(new_n254));
  nona32aa1n09x5               g159(.a(new_n187), .b(new_n249), .c(new_n254), .d(new_n213), .out0(new_n255));
  xorc02aa1n12x5               g160(.a(\a[25] ), .b(\b[24] ), .out0(new_n256));
  xnbna2aa1n03x5               g161(.a(new_n256), .b(new_n255), .c(new_n253), .out0(\s[25] ));
  nand02aa1d04x5               g162(.a(new_n255), .b(new_n253), .o1(new_n258));
  norp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  xnrc02aa1n12x5               g164(.a(\b[25] ), .b(\a[26] ), .out0(new_n260));
  aoai13aa1n03x5               g165(.a(new_n260), .b(new_n259), .c(new_n258), .d(new_n256), .o1(new_n261));
  nanp02aa1n04x5               g166(.a(new_n258), .b(new_n256), .o1(new_n262));
  nona22aa1n02x4               g167(.a(new_n262), .b(new_n260), .c(new_n259), .out0(new_n263));
  nanp02aa1n03x5               g168(.a(new_n263), .b(new_n261), .o1(\s[26] ));
  norb02aa1n03x5               g169(.a(new_n256), .b(new_n260), .out0(new_n265));
  inv000aa1d42x5               g170(.a(\a[26] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(\b[25] ), .o1(new_n267));
  tech160nm_fioaoi03aa1n03p5x5 g172(.a(new_n266), .b(new_n267), .c(new_n259), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  aoi012aa1n09x5               g174(.a(new_n269), .b(new_n252), .c(new_n265), .o1(new_n270));
  nona23aa1d18x5               g175(.a(new_n256), .b(new_n238), .c(new_n260), .d(new_n239), .out0(new_n271));
  nona32aa1n03x5               g176(.a(new_n187), .b(new_n271), .c(new_n254), .d(new_n213), .out0(new_n272));
  xorc02aa1n12x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n270), .c(new_n272), .out0(\s[27] ));
  nanp02aa1n03x5               g179(.a(new_n270), .b(new_n272), .o1(new_n275));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  norp02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .o1(new_n277));
  nand42aa1n03x5               g182(.a(\b[27] ), .b(\a[28] ), .o1(new_n278));
  nanb02aa1n06x5               g183(.a(new_n277), .b(new_n278), .out0(new_n279));
  aoai13aa1n03x5               g184(.a(new_n279), .b(new_n276), .c(new_n275), .d(new_n273), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n248), .b(new_n233), .c(new_n217), .d(new_n228), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n265), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n268), .b(new_n282), .c(new_n281), .d(new_n251), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n271), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n273), .b(new_n283), .c(new_n241), .d(new_n284), .o1(new_n285));
  nona22aa1n02x5               g190(.a(new_n285), .b(new_n279), .c(new_n276), .out0(new_n286));
  nanp02aa1n03x5               g191(.a(new_n280), .b(new_n286), .o1(\s[28] ));
  norb02aa1n03x5               g192(.a(new_n273), .b(new_n279), .out0(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n283), .c(new_n241), .d(new_n284), .o1(new_n289));
  norp02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .o1(new_n290));
  nand42aa1n03x5               g195(.a(\b[28] ), .b(\a[29] ), .o1(new_n291));
  norb02aa1n06x4               g196(.a(new_n291), .b(new_n290), .out0(new_n292));
  oai022aa1n02x5               g197(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n293));
  aboi22aa1n03x5               g198(.a(new_n290), .b(new_n291), .c(new_n293), .d(new_n278), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n288), .o1(new_n295));
  oai012aa1n02x5               g200(.a(new_n278), .b(new_n277), .c(new_n276), .o1(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n295), .c(new_n270), .d(new_n272), .o1(new_n297));
  aoi022aa1n03x5               g202(.a(new_n297), .b(new_n292), .c(new_n289), .d(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g204(.a(new_n279), .b(new_n273), .c(new_n292), .out0(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n283), .c(new_n241), .d(new_n284), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .out0(new_n302));
  aoi113aa1n02x5               g207(.a(new_n302), .b(new_n290), .c(new_n291), .d(new_n293), .e(new_n278), .o1(new_n303));
  inv000aa1n02x5               g208(.a(new_n300), .o1(new_n304));
  aoi013aa1n02x4               g209(.a(new_n290), .b(new_n293), .c(new_n291), .d(new_n278), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n304), .c(new_n270), .d(new_n272), .o1(new_n306));
  aoi022aa1n03x5               g211(.a(new_n306), .b(new_n302), .c(new_n301), .d(new_n303), .o1(\s[30] ));
  nand03aa1n02x5               g212(.a(new_n288), .b(new_n292), .c(new_n302), .o1(new_n308));
  nanb02aa1n02x5               g213(.a(new_n308), .b(new_n275), .out0(new_n309));
  xorc02aa1n02x5               g214(.a(\a[31] ), .b(\b[30] ), .out0(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n311));
  norb02aa1n02x5               g216(.a(new_n311), .b(new_n310), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n311), .b(new_n308), .c(new_n270), .d(new_n272), .o1(new_n313));
  aoi022aa1n02x7               g218(.a(new_n309), .b(new_n312), .c(new_n313), .d(new_n310), .o1(\s[31] ));
  xobna2aa1n03x5               g219(.a(new_n101), .b(new_n104), .c(new_n102), .out0(\s[3] ));
  nanb02aa1n02x5               g220(.a(\b[3] ), .b(new_n105), .out0(new_n316));
  aoi012aa1n02x5               g221(.a(new_n101), .b(new_n104), .c(new_n102), .o1(new_n317));
  aoi122aa1n02x5               g222(.a(new_n317), .b(new_n112), .c(new_n316), .d(new_n98), .e(new_n97), .o1(new_n318));
  aoi013aa1n02x4               g223(.a(new_n318), .b(new_n112), .c(new_n107), .d(new_n316), .o1(\s[4] ));
  xnbna2aa1n03x5               g224(.a(new_n115), .b(new_n107), .c(new_n112), .out0(\s[5] ));
  aoi013aa1n03x5               g225(.a(new_n113), .b(new_n107), .c(new_n112), .d(new_n114), .o1(new_n321));
  xnbna2aa1n03x5               g226(.a(new_n321), .b(new_n108), .c(new_n111), .out0(\s[6] ));
  oaoi03aa1n02x5               g227(.a(\a[6] ), .b(\b[5] ), .c(new_n321), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g229(.a(new_n119), .b(new_n323), .c(new_n120), .o1(new_n325));
  xnrb03aa1n03x5               g230(.a(new_n325), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g231(.a(new_n146), .b(new_n144), .c(new_n143), .out0(\s[9] ));
endmodule


