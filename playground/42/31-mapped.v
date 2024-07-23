// Benchmark "adder" written by ABC on Thu Jul 18 09:45:30 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n306, new_n307, new_n309, new_n312,
    new_n314, new_n316, new_n317;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n20x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nand22aa1n12x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  and002aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o(new_n101));
  inv000aa1n02x5               g006(.a(new_n101), .o1(new_n102));
  nand22aa1n09x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nand42aa1n20x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  norp02aa1n24x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  norb03aa1n12x5               g010(.a(new_n104), .b(new_n103), .c(new_n105), .out0(new_n106));
  nor002aa1d32x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nand42aa1n20x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nanb03aa1d24x5               g013(.a(new_n107), .b(new_n108), .c(new_n104), .out0(new_n109));
  oab012aa1n06x5               g014(.a(new_n107), .b(\a[4] ), .c(\b[3] ), .out0(new_n110));
  tech160nm_fioai012aa1n04x5   g015(.a(new_n110), .b(new_n106), .c(new_n109), .o1(new_n111));
  nand02aa1d04x5               g016(.a(new_n111), .b(new_n102), .o1(new_n112));
  nor002aa1n16x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  tech160nm_finand02aa1n05x5   g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nor022aa1n16x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nand02aa1d16x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nona23aa1n09x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  xorc02aa1n12x5               g022(.a(\a[7] ), .b(\b[6] ), .out0(new_n118));
  xnrc02aa1n12x5               g023(.a(\b[7] ), .b(\a[8] ), .out0(new_n119));
  inv000aa1n02x5               g024(.a(new_n119), .o1(new_n120));
  nanb03aa1n03x5               g025(.a(new_n117), .b(new_n120), .c(new_n118), .out0(new_n121));
  oa0022aa1n06x5               g026(.a(\a[8] ), .b(\b[7] ), .c(\a[7] ), .d(\b[6] ), .o(new_n122));
  nanp02aa1n02x5               g027(.a(\b[6] ), .b(\a[7] ), .o1(new_n123));
  aoai13aa1n06x5               g028(.a(new_n123), .b(new_n115), .c(new_n113), .d(new_n116), .o1(new_n124));
  aoi022aa1d18x5               g029(.a(new_n124), .b(new_n122), .c(\a[8] ), .d(\b[7] ), .o1(new_n125));
  inv040aa1n02x5               g030(.a(new_n125), .o1(new_n126));
  nor042aa1n09x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n100), .b(new_n127), .out0(new_n128));
  oai112aa1n04x5               g033(.a(new_n126), .b(new_n128), .c(new_n112), .d(new_n121), .o1(new_n129));
  xobna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n100), .out0(\s[10] ));
  nor002aa1d32x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  nand02aa1d28x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  aoai13aa1n06x5               g038(.a(new_n98), .b(new_n97), .c(new_n129), .d(new_n100), .o1(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n132), .c(new_n133), .out0(\s[11] ));
  nanb02aa1d24x5               g040(.a(new_n131), .b(new_n133), .out0(new_n136));
  norp02aa1n24x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand02aa1d12x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanb02aa1n03x5               g043(.a(new_n137), .b(new_n138), .out0(new_n139));
  oaoi13aa1n02x7               g044(.a(new_n139), .b(new_n132), .c(new_n134), .d(new_n136), .o1(new_n140));
  oai112aa1n02x7               g045(.a(new_n132), .b(new_n139), .c(new_n134), .d(new_n136), .o1(new_n141));
  norb02aa1n03x4               g046(.a(new_n141), .b(new_n140), .out0(\s[12] ));
  oaoi13aa1n12x5               g047(.a(new_n101), .b(new_n110), .c(new_n106), .d(new_n109), .o1(new_n143));
  norb03aa1n06x5               g048(.a(new_n118), .b(new_n117), .c(new_n119), .out0(new_n144));
  nano23aa1n03x5               g049(.a(new_n127), .b(new_n97), .c(new_n98), .d(new_n100), .out0(new_n145));
  nona22aa1n03x5               g050(.a(new_n145), .b(new_n139), .c(new_n136), .out0(new_n146));
  inv000aa1n02x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n125), .c(new_n143), .d(new_n144), .o1(new_n148));
  nona23aa1n09x5               g053(.a(new_n138), .b(new_n133), .c(new_n131), .d(new_n137), .out0(new_n149));
  aoi012aa1n06x5               g054(.a(new_n97), .b(new_n127), .c(new_n98), .o1(new_n150));
  tech160nm_fiaoi012aa1n03p5x5 g055(.a(new_n137), .b(new_n131), .c(new_n138), .o1(new_n151));
  oai012aa1n12x5               g056(.a(new_n151), .b(new_n149), .c(new_n150), .o1(new_n152));
  inv000aa1n02x5               g057(.a(new_n152), .o1(new_n153));
  xnrc02aa1n12x5               g058(.a(\b[12] ), .b(\a[13] ), .out0(new_n154));
  inv040aa1n02x5               g059(.a(new_n154), .o1(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n155), .b(new_n148), .c(new_n153), .out0(\s[13] ));
  orn002aa1n24x5               g061(.a(\a[13] ), .b(\b[12] ), .o(new_n157));
  tech160nm_fioai012aa1n05x5   g062(.a(new_n126), .b(new_n112), .c(new_n121), .o1(new_n158));
  aoai13aa1n03x5               g063(.a(new_n155), .b(new_n152), .c(new_n158), .d(new_n147), .o1(new_n159));
  xorc02aa1n12x5               g064(.a(\a[14] ), .b(\b[13] ), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n159), .c(new_n157), .out0(\s[14] ));
  norb02aa1n09x5               g066(.a(new_n160), .b(new_n154), .out0(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  oaoi03aa1n12x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n157), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n04x5               g070(.a(new_n165), .b(new_n163), .c(new_n148), .d(new_n153), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n09x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1d28x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nor042aa1n09x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1n16x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  aoi112aa1n03x4               g077(.a(new_n168), .b(new_n172), .c(new_n166), .d(new_n169), .o1(new_n173));
  aoai13aa1n03x5               g078(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n174));
  norb02aa1n02x7               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  nano23aa1d15x5               g080(.a(new_n168), .b(new_n170), .c(new_n171), .d(new_n169), .out0(new_n176));
  nano32aa1n03x7               g081(.a(new_n146), .b(new_n176), .c(new_n155), .d(new_n160), .out0(new_n177));
  aoai13aa1n12x5               g082(.a(new_n177), .b(new_n125), .c(new_n143), .d(new_n144), .o1(new_n178));
  aoi012aa1n02x7               g083(.a(new_n170), .b(new_n168), .c(new_n171), .o1(new_n179));
  aob012aa1n12x5               g084(.a(new_n179), .b(new_n176), .c(new_n164), .out0(new_n180));
  aoi013aa1n09x5               g085(.a(new_n180), .b(new_n152), .c(new_n162), .d(new_n176), .o1(new_n181));
  nand02aa1d08x5               g086(.a(new_n178), .b(new_n181), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g088(.a(\a[18] ), .o1(new_n184));
  inv040aa1d32x5               g089(.a(\a[17] ), .o1(new_n185));
  inv020aa1n10x5               g090(.a(\b[16] ), .o1(new_n186));
  oaoi03aa1n03x5               g091(.a(new_n185), .b(new_n186), .c(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n184), .out0(\s[18] ));
  xroi22aa1d06x4               g093(.a(new_n185), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n189));
  nand02aa1d16x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nona22aa1n03x5               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(new_n191));
  oaib12aa1n09x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n184), .out0(new_n192));
  nor002aa1d32x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nand02aa1n08x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n192), .c(new_n182), .d(new_n189), .o1(new_n196));
  aoi112aa1n03x4               g101(.a(new_n195), .b(new_n192), .c(new_n182), .d(new_n189), .o1(new_n197));
  norb02aa1n02x7               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand42aa1n16x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nona22aa1n02x5               g107(.a(new_n196), .b(new_n202), .c(new_n193), .out0(new_n203));
  orn002aa1n02x5               g108(.a(\a[19] ), .b(\b[18] ), .o(new_n204));
  aobi12aa1n06x5               g109(.a(new_n202), .b(new_n196), .c(new_n204), .out0(new_n205));
  norb02aa1n03x4               g110(.a(new_n203), .b(new_n205), .out0(\s[20] ));
  nano23aa1n09x5               g111(.a(new_n193), .b(new_n200), .c(new_n201), .d(new_n194), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n189), .b(new_n207), .o1(new_n208));
  nor042aa1n02x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  aoi013aa1n06x4               g114(.a(new_n209), .b(new_n190), .c(new_n185), .d(new_n186), .o1(new_n210));
  nona23aa1n09x5               g115(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n211));
  aoi012aa1n09x5               g116(.a(new_n200), .b(new_n193), .c(new_n201), .o1(new_n212));
  oai012aa1n18x5               g117(.a(new_n212), .b(new_n211), .c(new_n210), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n04x5               g119(.a(new_n214), .b(new_n208), .c(new_n178), .d(new_n181), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n08x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  tech160nm_finand02aa1n03p5x5 g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  nor042aa1n04x5               g124(.a(\b[21] ), .b(\a[22] ), .o1(new_n220));
  nand02aa1n08x5               g125(.a(\b[21] ), .b(\a[22] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  aoi112aa1n02x5               g127(.a(new_n217), .b(new_n222), .c(new_n215), .d(new_n219), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n222), .b(new_n217), .c(new_n215), .d(new_n219), .o1(new_n224));
  norb02aa1n03x4               g129(.a(new_n224), .b(new_n223), .out0(\s[22] ));
  nano23aa1n06x5               g130(.a(new_n217), .b(new_n220), .c(new_n221), .d(new_n218), .out0(new_n226));
  nanp03aa1n06x5               g131(.a(new_n189), .b(new_n207), .c(new_n226), .o1(new_n227));
  aoi012aa1d18x5               g132(.a(new_n220), .b(new_n217), .c(new_n221), .o1(new_n228));
  inv000aa1n02x5               g133(.a(new_n228), .o1(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n213), .c(new_n226), .o1(new_n230));
  aoai13aa1n04x5               g135(.a(new_n230), .b(new_n227), .c(new_n178), .d(new_n181), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n12x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n12x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n02x7               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  inv000aa1n02x5               g143(.a(new_n212), .o1(new_n239));
  aoai13aa1n06x5               g144(.a(new_n226), .b(new_n239), .c(new_n207), .d(new_n192), .o1(new_n240));
  and002aa1n02x5               g145(.a(new_n235), .b(new_n234), .o(new_n241));
  inv000aa1n02x5               g146(.a(new_n241), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n243));
  oab012aa1n06x5               g148(.a(new_n243), .b(\a[24] ), .c(\b[23] ), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n242), .c(new_n240), .d(new_n228), .o1(new_n245));
  nano32aa1n02x5               g150(.a(new_n208), .b(new_n235), .c(new_n226), .d(new_n234), .out0(new_n246));
  xorc02aa1n02x5               g151(.a(\a[25] ), .b(\b[24] ), .out0(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n245), .c(new_n182), .d(new_n246), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n245), .b(new_n247), .c(new_n182), .d(new_n246), .o1(new_n249));
  norb02aa1n02x7               g154(.a(new_n248), .b(new_n249), .out0(\s[25] ));
  nor042aa1n03x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[26] ), .b(\b[25] ), .out0(new_n252));
  nona22aa1n02x5               g157(.a(new_n248), .b(new_n252), .c(new_n251), .out0(new_n253));
  inv000aa1n02x5               g158(.a(new_n251), .o1(new_n254));
  aobi12aa1n06x5               g159(.a(new_n252), .b(new_n248), .c(new_n254), .out0(new_n255));
  norb02aa1n03x4               g160(.a(new_n253), .b(new_n255), .out0(\s[26] ));
  nanp03aa1n02x5               g161(.a(new_n155), .b(new_n176), .c(new_n160), .o1(new_n257));
  oabi12aa1n06x5               g162(.a(new_n180), .b(new_n153), .c(new_n257), .out0(new_n258));
  inv040aa1d30x5               g163(.a(\a[25] ), .o1(new_n259));
  inv000aa1n06x5               g164(.a(\a[26] ), .o1(new_n260));
  xroi22aa1d06x4               g165(.a(new_n259), .b(\b[24] ), .c(new_n260), .d(\b[25] ), .out0(new_n261));
  nano22aa1n03x7               g166(.a(new_n227), .b(new_n241), .c(new_n261), .out0(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n258), .c(new_n158), .d(new_n177), .o1(new_n263));
  oao003aa1n03x5               g168(.a(\a[26] ), .b(\b[25] ), .c(new_n254), .carry(new_n264));
  aobi12aa1n06x5               g169(.a(new_n264), .b(new_n245), .c(new_n261), .out0(new_n265));
  xorc02aa1n12x5               g170(.a(\a[27] ), .b(\b[26] ), .out0(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n263), .out0(\s[27] ));
  nor042aa1n03x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  inv040aa1n03x5               g173(.a(new_n268), .o1(new_n269));
  aobi12aa1n02x7               g174(.a(new_n266), .b(new_n265), .c(new_n263), .out0(new_n270));
  xnrc02aa1n12x5               g175(.a(\b[27] ), .b(\a[28] ), .out0(new_n271));
  nano22aa1n03x5               g176(.a(new_n270), .b(new_n269), .c(new_n271), .out0(new_n272));
  inv000aa1n02x5               g177(.a(new_n262), .o1(new_n273));
  aoi012aa1n06x5               g178(.a(new_n273), .b(new_n178), .c(new_n181), .o1(new_n274));
  aoai13aa1n02x5               g179(.a(new_n241), .b(new_n229), .c(new_n213), .d(new_n226), .o1(new_n275));
  inv000aa1n02x5               g180(.a(new_n261), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n264), .b(new_n276), .c(new_n275), .d(new_n244), .o1(new_n277));
  oaih12aa1n02x5               g182(.a(new_n266), .b(new_n277), .c(new_n274), .o1(new_n278));
  tech160nm_fiaoi012aa1n02p5x5 g183(.a(new_n271), .b(new_n278), .c(new_n269), .o1(new_n279));
  norp02aa1n03x5               g184(.a(new_n279), .b(new_n272), .o1(\s[28] ));
  xnrc02aa1n12x5               g185(.a(\b[28] ), .b(\a[29] ), .out0(new_n281));
  norb02aa1n02x5               g186(.a(new_n266), .b(new_n271), .out0(new_n282));
  aobi12aa1n02x7               g187(.a(new_n282), .b(new_n265), .c(new_n263), .out0(new_n283));
  oao003aa1n03x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n269), .carry(new_n284));
  nano22aa1n02x4               g189(.a(new_n283), .b(new_n281), .c(new_n284), .out0(new_n285));
  oaih12aa1n02x5               g190(.a(new_n282), .b(new_n277), .c(new_n274), .o1(new_n286));
  tech160nm_fiaoi012aa1n02p5x5 g191(.a(new_n281), .b(new_n286), .c(new_n284), .o1(new_n287));
  norp02aa1n03x5               g192(.a(new_n287), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g194(.a(new_n266), .b(new_n281), .c(new_n271), .out0(new_n290));
  aobi12aa1n02x7               g195(.a(new_n290), .b(new_n265), .c(new_n263), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n291), .b(new_n292), .c(new_n293), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n290), .b(new_n277), .c(new_n274), .o1(new_n295));
  tech160nm_fiaoi012aa1n02p5x5 g200(.a(new_n293), .b(new_n295), .c(new_n292), .o1(new_n296));
  norp02aa1n03x5               g201(.a(new_n296), .b(new_n294), .o1(\s[30] ));
  norb02aa1n02x5               g202(.a(new_n290), .b(new_n293), .out0(new_n298));
  aobi12aa1n02x7               g203(.a(new_n298), .b(new_n265), .c(new_n263), .out0(new_n299));
  oao003aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .c(new_n292), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n299), .b(new_n300), .c(new_n301), .out0(new_n302));
  oaih12aa1n02x5               g207(.a(new_n298), .b(new_n277), .c(new_n274), .o1(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n301), .b(new_n303), .c(new_n300), .o1(new_n304));
  norp02aa1n03x5               g209(.a(new_n304), .b(new_n302), .o1(\s[31] ));
  norb02aa1n02x5               g210(.a(new_n108), .b(new_n107), .out0(new_n306));
  oaoi13aa1n02x5               g211(.a(new_n306), .b(new_n104), .c(new_n103), .d(new_n105), .o1(new_n307));
  oab012aa1n02x4               g212(.a(new_n307), .b(new_n106), .c(new_n109), .out0(\s[3] ));
  oabi12aa1n02x5               g213(.a(new_n107), .b(new_n106), .c(new_n109), .out0(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n143), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi013aa1n03x5               g216(.a(new_n113), .b(new_n111), .c(new_n102), .d(new_n114), .o1(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb03aa1n02x5               g218(.a(new_n115), .b(new_n312), .c(new_n116), .out0(new_n314));
  xobna2aa1n03x5               g219(.a(new_n118), .b(new_n314), .c(new_n116), .out0(\s[7] ));
  orn002aa1n02x5               g220(.a(\a[7] ), .b(\b[6] ), .o(new_n316));
  nanp03aa1n03x5               g221(.a(new_n314), .b(new_n116), .c(new_n118), .o1(new_n317));
  xnbna2aa1n03x5               g222(.a(new_n120), .b(new_n317), .c(new_n316), .out0(\s[8] ));
  xorb03aa1n02x5               g223(.a(new_n158), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


