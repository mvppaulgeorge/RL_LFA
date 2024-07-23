// Benchmark "adder" written by ABC on Thu Jul 18 02:07:28 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n336, new_n338,
    new_n341, new_n342, new_n344, new_n345, new_n347, new_n348;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand22aa1n04x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand42aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor042aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  oai112aa1n06x5               g007(.a(new_n102), .b(new_n100), .c(new_n101), .d(new_n99), .o1(new_n103));
  oa0022aa1n02x5               g008(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n104));
  aoi022aa1n12x5               g009(.a(new_n103), .b(new_n104), .c(\b[3] ), .d(\a[4] ), .o1(new_n105));
  nor002aa1n03x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  nand42aa1n02x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  norp02aa1n06x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  nand42aa1n02x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nona23aa1n09x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  norp02aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nand42aa1n06x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  norb02aa1n02x7               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nor042aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand42aa1n03x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  norb02aa1n02x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nano22aa1n06x5               g021(.a(new_n110), .b(new_n113), .c(new_n116), .out0(new_n117));
  aoi012aa1n02x7               g022(.a(new_n111), .b(new_n114), .c(new_n112), .o1(new_n118));
  tech160nm_fiao0012aa1n02p5x5 g023(.a(new_n106), .b(new_n108), .c(new_n107), .o(new_n119));
  oabi12aa1n09x5               g024(.a(new_n119), .b(new_n110), .c(new_n118), .out0(new_n120));
  nand42aa1n02x5               g025(.a(\b[8] ), .b(\a[9] ), .o1(new_n121));
  norb02aa1n02x5               g026(.a(new_n121), .b(new_n97), .out0(new_n122));
  aoai13aa1n06x5               g027(.a(new_n122), .b(new_n120), .c(new_n105), .d(new_n117), .o1(new_n123));
  nor042aa1n06x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nanp02aa1n04x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n123), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g032(.a(new_n124), .o1(new_n128));
  inv000aa1n02x5               g033(.a(new_n126), .o1(new_n129));
  aoai13aa1n04x5               g034(.a(new_n128), .b(new_n129), .c(new_n123), .d(new_n98), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n04x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand42aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  norp02aa1n04x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nand42aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  inv000aa1d42x5               g042(.a(new_n137), .o1(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n132), .c(new_n130), .d(new_n134), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(new_n130), .b(new_n134), .o1(new_n140));
  nona22aa1n02x4               g045(.a(new_n140), .b(new_n138), .c(new_n132), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n141), .b(new_n139), .o1(\s[12] ));
  nano23aa1n06x5               g047(.a(new_n132), .b(new_n135), .c(new_n136), .d(new_n133), .out0(new_n143));
  nano23aa1n06x5               g048(.a(new_n97), .b(new_n124), .c(new_n125), .d(new_n121), .out0(new_n144));
  nand22aa1n09x5               g049(.a(new_n144), .b(new_n143), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  aoai13aa1n06x5               g051(.a(new_n146), .b(new_n120), .c(new_n105), .d(new_n117), .o1(new_n147));
  oai012aa1n12x5               g052(.a(new_n125), .b(new_n124), .c(new_n97), .o1(new_n148));
  tech160nm_fiao0012aa1n02p5x5 g053(.a(new_n135), .b(new_n132), .c(new_n136), .o(new_n149));
  aoib12aa1n02x5               g054(.a(new_n149), .b(new_n143), .c(new_n148), .out0(new_n150));
  norp02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  norb02aa1n03x5               g057(.a(new_n152), .b(new_n151), .out0(new_n153));
  xnbna2aa1n03x5               g058(.a(new_n153), .b(new_n147), .c(new_n150), .out0(\s[13] ));
  inv000aa1d42x5               g059(.a(\a[13] ), .o1(new_n155));
  inv000aa1d42x5               g060(.a(\b[12] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(new_n156), .b(new_n155), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[3] ), .b(\a[4] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(new_n103), .b(new_n104), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(new_n159), .b(new_n158), .o1(new_n160));
  nano23aa1n03x7               g065(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n161));
  nanp03aa1n02x5               g066(.a(new_n161), .b(new_n113), .c(new_n116), .o1(new_n162));
  aoib12aa1n02x5               g067(.a(new_n119), .b(new_n161), .c(new_n118), .out0(new_n163));
  oai012aa1n02x5               g068(.a(new_n163), .b(new_n160), .c(new_n162), .o1(new_n164));
  nona23aa1n03x5               g069(.a(new_n136), .b(new_n133), .c(new_n132), .d(new_n135), .out0(new_n165));
  oabi12aa1n06x5               g070(.a(new_n149), .b(new_n165), .c(new_n148), .out0(new_n166));
  aoai13aa1n02x5               g071(.a(new_n153), .b(new_n166), .c(new_n164), .d(new_n146), .o1(new_n167));
  nor042aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand42aa1n06x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  norb02aa1n03x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n170), .b(new_n167), .c(new_n157), .out0(\s[14] ));
  nona23aa1n09x5               g076(.a(new_n169), .b(new_n152), .c(new_n151), .d(new_n168), .out0(new_n172));
  aoai13aa1n06x5               g077(.a(new_n169), .b(new_n168), .c(new_n155), .d(new_n156), .o1(new_n173));
  aoai13aa1n04x5               g078(.a(new_n173), .b(new_n172), .c(new_n147), .d(new_n150), .o1(new_n174));
  xorb03aa1n02x5               g079(.a(new_n174), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nand02aa1n08x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  norp02aa1n04x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanp02aa1n04x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanb02aa1n02x5               g084(.a(new_n178), .b(new_n179), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n176), .c(new_n174), .d(new_n177), .o1(new_n181));
  norb02aa1n06x4               g086(.a(new_n177), .b(new_n176), .out0(new_n182));
  nand42aa1n03x5               g087(.a(new_n174), .b(new_n182), .o1(new_n183));
  nona22aa1n02x4               g088(.a(new_n183), .b(new_n180), .c(new_n176), .out0(new_n184));
  nanp02aa1n02x5               g089(.a(new_n184), .b(new_n181), .o1(\s[16] ));
  nano23aa1n06x5               g090(.a(new_n176), .b(new_n178), .c(new_n179), .d(new_n177), .out0(new_n186));
  nano32aa1d12x5               g091(.a(new_n145), .b(new_n186), .c(new_n153), .d(new_n170), .out0(new_n187));
  aoai13aa1n12x5               g092(.a(new_n187), .b(new_n120), .c(new_n105), .d(new_n117), .o1(new_n188));
  inv000aa1n02x5               g093(.a(new_n178), .o1(new_n189));
  nano32aa1n03x7               g094(.a(new_n172), .b(new_n179), .c(new_n182), .d(new_n189), .out0(new_n190));
  inv000aa1n02x5               g095(.a(new_n176), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n177), .o1(new_n192));
  aoai13aa1n02x7               g097(.a(new_n189), .b(new_n192), .c(new_n173), .d(new_n191), .o1(new_n193));
  aoi022aa1n12x5               g098(.a(new_n166), .b(new_n190), .c(new_n193), .d(new_n179), .o1(new_n194));
  xnrc02aa1n03x5               g099(.a(\b[16] ), .b(\a[17] ), .out0(new_n195));
  xobna2aa1n03x5               g100(.a(new_n195), .b(new_n188), .c(new_n194), .out0(\s[17] ));
  norp02aa1n02x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  inv000aa1d42x5               g103(.a(\a[17] ), .o1(new_n199));
  aoi012aa1n12x5               g104(.a(new_n120), .b(new_n105), .c(new_n117), .o1(new_n200));
  nona23aa1n06x5               g105(.a(new_n186), .b(new_n144), .c(new_n165), .d(new_n172), .out0(new_n201));
  oai012aa1n18x5               g106(.a(new_n194), .b(new_n200), .c(new_n201), .o1(new_n202));
  oaib12aa1n06x5               g107(.a(new_n202), .b(new_n199), .c(\b[16] ), .out0(new_n203));
  tech160nm_fixorc02aa1n03p5x5 g108(.a(\a[18] ), .b(\b[17] ), .out0(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n203), .c(new_n198), .out0(\s[18] ));
  inv000aa1d42x5               g110(.a(\a[18] ), .o1(new_n206));
  xroi22aa1d04x5               g111(.a(new_n199), .b(\b[16] ), .c(new_n206), .d(\b[17] ), .out0(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  nand22aa1n03x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  oai022aa1d18x5               g114(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(new_n210), .b(new_n209), .o1(new_n211));
  aoai13aa1n04x5               g116(.a(new_n211), .b(new_n208), .c(new_n188), .d(new_n194), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nand02aa1n06x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  xnrc02aa1n02x5               g121(.a(\b[19] ), .b(\a[20] ), .out0(new_n217));
  aoai13aa1n03x5               g122(.a(new_n217), .b(new_n215), .c(new_n212), .d(new_n216), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n211), .o1(new_n219));
  nanb02aa1n02x5               g124(.a(new_n215), .b(new_n216), .out0(new_n220));
  inv040aa1n02x5               g125(.a(new_n220), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n219), .c(new_n202), .d(new_n207), .o1(new_n222));
  nona22aa1n03x5               g127(.a(new_n222), .b(new_n217), .c(new_n215), .out0(new_n223));
  nanp02aa1n03x5               g128(.a(new_n218), .b(new_n223), .o1(\s[20] ));
  nona23aa1d18x5               g129(.a(new_n221), .b(new_n204), .c(new_n217), .d(new_n195), .out0(new_n225));
  aoai13aa1n09x5               g130(.a(new_n216), .b(new_n215), .c(new_n210), .d(new_n209), .o1(new_n226));
  oaoi03aa1n12x5               g131(.a(\a[20] ), .b(\b[19] ), .c(new_n226), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n225), .c(new_n188), .d(new_n194), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  nand42aa1n04x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nanb02aa1n02x5               g137(.a(new_n231), .b(new_n232), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  nor042aa1n03x5               g139(.a(\b[21] ), .b(\a[22] ), .o1(new_n235));
  nand42aa1n06x5               g140(.a(\b[21] ), .b(\a[22] ), .o1(new_n236));
  nanb02aa1n02x5               g141(.a(new_n235), .b(new_n236), .out0(new_n237));
  aoai13aa1n03x5               g142(.a(new_n237), .b(new_n231), .c(new_n229), .d(new_n234), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n225), .o1(new_n239));
  aoai13aa1n06x5               g144(.a(new_n234), .b(new_n227), .c(new_n202), .d(new_n239), .o1(new_n240));
  nona22aa1n02x4               g145(.a(new_n240), .b(new_n237), .c(new_n231), .out0(new_n241));
  nanp02aa1n02x5               g146(.a(new_n238), .b(new_n241), .o1(\s[22] ));
  nano23aa1d15x5               g147(.a(new_n231), .b(new_n235), .c(new_n236), .d(new_n232), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  nor002aa1n02x5               g149(.a(new_n225), .b(new_n244), .o1(new_n245));
  inv000aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  tech160nm_fioai012aa1n03p5x5 g151(.a(new_n236), .b(new_n235), .c(new_n231), .o1(new_n247));
  aobi12aa1n18x5               g152(.a(new_n247), .b(new_n227), .c(new_n243), .out0(new_n248));
  aoai13aa1n02x7               g153(.a(new_n248), .b(new_n246), .c(new_n188), .d(new_n194), .o1(new_n249));
  xorb03aa1n02x5               g154(.a(new_n249), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n02x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n252), .b(new_n251), .out0(new_n253));
  nor042aa1n02x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(\b[23] ), .b(\a[24] ), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n255), .b(new_n254), .out0(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n251), .c(new_n249), .d(new_n253), .o1(new_n258));
  inv040aa1n02x5               g163(.a(new_n248), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n253), .b(new_n259), .c(new_n202), .d(new_n245), .o1(new_n260));
  nona22aa1n03x5               g165(.a(new_n260), .b(new_n257), .c(new_n251), .out0(new_n261));
  nanp02aa1n02x5               g166(.a(new_n258), .b(new_n261), .o1(\s[24] ));
  nano23aa1n06x5               g167(.a(new_n251), .b(new_n254), .c(new_n255), .d(new_n252), .out0(new_n263));
  nano22aa1d15x5               g168(.a(new_n225), .b(new_n243), .c(new_n263), .out0(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  nand02aa1d04x5               g170(.a(new_n263), .b(new_n243), .o1(new_n266));
  inv040aa1n04x5               g171(.a(new_n266), .o1(new_n267));
  oai012aa1n02x5               g172(.a(new_n255), .b(new_n254), .c(new_n251), .o1(new_n268));
  oaib12aa1n09x5               g173(.a(new_n268), .b(new_n247), .c(new_n263), .out0(new_n269));
  aoi012aa1d18x5               g174(.a(new_n269), .b(new_n227), .c(new_n267), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n265), .c(new_n188), .d(new_n194), .o1(new_n271));
  xorb03aa1n02x5               g176(.a(new_n271), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[25] ), .b(\b[24] ), .out0(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[25] ), .b(\a[26] ), .out0(new_n275));
  aoai13aa1n03x5               g180(.a(new_n275), .b(new_n273), .c(new_n271), .d(new_n274), .o1(new_n276));
  inv000aa1n02x5               g181(.a(new_n270), .o1(new_n277));
  aoai13aa1n03x5               g182(.a(new_n274), .b(new_n277), .c(new_n202), .d(new_n264), .o1(new_n278));
  nona22aa1n03x5               g183(.a(new_n278), .b(new_n275), .c(new_n273), .out0(new_n279));
  nanp02aa1n03x5               g184(.a(new_n276), .b(new_n279), .o1(\s[26] ));
  nanp03aa1n02x5               g185(.a(new_n186), .b(new_n153), .c(new_n170), .o1(new_n281));
  nanp02aa1n02x5               g186(.a(new_n193), .b(new_n179), .o1(new_n282));
  tech160nm_fioai012aa1n05x5   g187(.a(new_n282), .b(new_n150), .c(new_n281), .o1(new_n283));
  norb02aa1n02x7               g188(.a(new_n274), .b(new_n275), .out0(new_n284));
  nano32aa1n03x7               g189(.a(new_n225), .b(new_n284), .c(new_n243), .d(new_n263), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n283), .c(new_n164), .d(new_n187), .o1(new_n286));
  orn002aa1n02x5               g191(.a(\a[20] ), .b(\b[19] ), .o(new_n287));
  and002aa1n02x5               g192(.a(\b[19] ), .b(\a[20] ), .o(new_n288));
  aoi112aa1n02x5               g193(.a(new_n266), .b(new_n288), .c(new_n226), .d(new_n287), .o1(new_n289));
  inv000aa1d42x5               g194(.a(\a[26] ), .o1(new_n290));
  inv000aa1d42x5               g195(.a(\b[25] ), .o1(new_n291));
  oaoi03aa1n02x5               g196(.a(new_n290), .b(new_n291), .c(new_n273), .o1(new_n292));
  inv000aa1n02x5               g197(.a(new_n292), .o1(new_n293));
  oaoi13aa1n04x5               g198(.a(new_n293), .b(new_n284), .c(new_n289), .d(new_n269), .o1(new_n294));
  nor042aa1n04x5               g199(.a(\b[26] ), .b(\a[27] ), .o1(new_n295));
  and002aa1n18x5               g200(.a(\b[26] ), .b(\a[27] ), .o(new_n296));
  nor042aa1n04x5               g201(.a(new_n296), .b(new_n295), .o1(new_n297));
  xnbna2aa1n03x5               g202(.a(new_n297), .b(new_n286), .c(new_n294), .out0(\s[27] ));
  aoai13aa1n03x5               g203(.a(new_n284), .b(new_n269), .c(new_n227), .d(new_n267), .o1(new_n299));
  tech160nm_finand02aa1n05x5   g204(.a(new_n299), .b(new_n292), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n296), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n300), .c(new_n202), .d(new_n285), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n295), .o1(new_n303));
  aoai13aa1n02x5               g208(.a(new_n303), .b(new_n296), .c(new_n286), .d(new_n294), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[28] ), .b(\b[27] ), .out0(new_n305));
  norp02aa1n02x5               g210(.a(new_n305), .b(new_n295), .o1(new_n306));
  aoi022aa1n03x5               g211(.a(new_n304), .b(new_n305), .c(new_n302), .d(new_n306), .o1(\s[28] ));
  and002aa1n06x5               g212(.a(new_n305), .b(new_n297), .o(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n300), .c(new_n202), .d(new_n285), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n308), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n303), .carry(new_n311));
  aoai13aa1n02x5               g216(.a(new_n311), .b(new_n310), .c(new_n286), .d(new_n294), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n311), .b(new_n313), .out0(new_n314));
  aoi022aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n309), .d(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g221(.a(new_n305), .b(new_n313), .c(new_n297), .o(new_n317));
  aoai13aa1n06x5               g222(.a(new_n317), .b(new_n300), .c(new_n202), .d(new_n285), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n317), .o1(new_n319));
  oaoi03aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .o1(new_n320));
  inv000aa1n03x5               g225(.a(new_n320), .o1(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n319), .c(new_n286), .d(new_n294), .o1(new_n322));
  xorc02aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .out0(new_n323));
  and002aa1n02x5               g228(.a(\b[28] ), .b(\a[29] ), .o(new_n324));
  oabi12aa1n02x5               g229(.a(new_n323), .b(\a[29] ), .c(\b[28] ), .out0(new_n325));
  oab012aa1n02x4               g230(.a(new_n325), .b(new_n311), .c(new_n324), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n322), .b(new_n323), .c(new_n318), .d(new_n326), .o1(\s[30] ));
  nanp03aa1n02x5               g232(.a(new_n308), .b(new_n313), .c(new_n323), .o1(new_n328));
  inv000aa1n02x5               g233(.a(new_n328), .o1(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n300), .c(new_n202), .d(new_n285), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[31] ), .b(\b[30] ), .out0(new_n331));
  oao003aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .c(new_n321), .carry(new_n332));
  norb02aa1n02x5               g237(.a(new_n332), .b(new_n331), .out0(new_n333));
  aoai13aa1n02x7               g238(.a(new_n332), .b(new_n328), .c(new_n286), .d(new_n294), .o1(new_n334));
  aoi022aa1n03x5               g239(.a(new_n334), .b(new_n331), .c(new_n330), .d(new_n333), .o1(\s[31] ));
  oai012aa1n02x5               g240(.a(new_n100), .b(new_n101), .c(new_n99), .o1(new_n336));
  xnrb03aa1n02x5               g241(.a(new_n336), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g242(.a(\a[3] ), .b(\b[2] ), .c(new_n336), .o1(new_n338));
  xorb03aa1n02x5               g243(.a(new_n338), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g244(.a(new_n105), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi13aa1n02x5               g245(.a(new_n113), .b(new_n115), .c(new_n105), .d(new_n114), .o1(new_n341));
  oai112aa1n03x5               g246(.a(new_n115), .b(new_n113), .c(new_n105), .d(new_n114), .o1(new_n342));
  norb02aa1n02x5               g247(.a(new_n342), .b(new_n341), .out0(\s[6] ));
  norb02aa1n02x5               g248(.a(new_n109), .b(new_n108), .out0(new_n344));
  orn002aa1n02x5               g249(.a(\a[6] ), .b(\b[5] ), .o(new_n345));
  xnbna2aa1n03x5               g250(.a(new_n344), .b(new_n342), .c(new_n345), .out0(\s[7] ));
  oai012aa1n02x5               g251(.a(new_n342), .b(\b[5] ), .c(\a[6] ), .o1(new_n347));
  aoi012aa1n02x5               g252(.a(new_n108), .b(new_n347), .c(new_n109), .o1(new_n348));
  xnrb03aa1n03x5               g253(.a(new_n348), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g254(.a(new_n200), .b(new_n121), .c(new_n98), .out0(\s[9] ));
endmodule


