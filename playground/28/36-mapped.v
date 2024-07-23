// Benchmark "adder" written by ABC on Thu Jul 18 02:38:11 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n322, new_n325, new_n326,
    new_n327, new_n329, new_n330, new_n331, new_n332, new_n333, new_n335;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nanp02aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1d04x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nor042aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  tech160nm_fioai012aa1n03p5x5 g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  nor022aa1n04x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nor022aa1n16x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand42aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nona23aa1n03x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n02x7               g010(.a(new_n103), .b(new_n101), .c(new_n104), .o1(new_n106));
  oai012aa1n09x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  nor002aa1d32x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  nanp02aa1n04x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nand22aa1n12x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor042aa1n06x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nona23aa1d18x5               g016(.a(new_n110), .b(new_n109), .c(new_n111), .d(new_n108), .out0(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .out0(new_n113));
  tech160nm_fixnrc02aa1n04x5   g018(.a(\b[5] ), .b(\a[6] ), .out0(new_n114));
  nor043aa1n06x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  ao0012aa1n03x7               g020(.a(new_n111), .b(new_n108), .c(new_n110), .o(new_n116));
  inv000aa1d42x5               g021(.a(\b[5] ), .o1(new_n117));
  oaih22aa1n04x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  oaib12aa1n06x5               g023(.a(new_n118), .b(new_n117), .c(\a[6] ), .out0(new_n119));
  oabi12aa1n18x5               g024(.a(new_n116), .b(new_n112), .c(new_n119), .out0(new_n120));
  nor042aa1n06x5               g025(.a(\b[8] ), .b(\a[9] ), .o1(new_n121));
  nanp02aa1n12x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  norb02aa1n02x5               g027(.a(new_n122), .b(new_n121), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n120), .c(new_n107), .d(new_n115), .o1(new_n124));
  oai012aa1n02x5               g029(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .o1(new_n125));
  xorb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1d28x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nor002aa1d32x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nand42aa1d28x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanb02aa1n02x5               g034(.a(new_n128), .b(new_n129), .out0(new_n130));
  oai022aa1d24x5               g035(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(new_n124), .b(new_n132), .o1(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n130), .b(new_n133), .c(new_n127), .out0(\s[11] ));
  inv000aa1d42x5               g039(.a(\b[11] ), .o1(new_n135));
  nanb03aa1n12x5               g040(.a(new_n128), .b(new_n129), .c(new_n127), .out0(new_n136));
  aoib12aa1n02x5               g041(.a(new_n128), .b(new_n133), .c(new_n136), .out0(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(new_n135), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n04x5               g043(.a(\b[9] ), .b(\a[10] ), .o1(new_n139));
  nano23aa1d15x5               g044(.a(new_n139), .b(new_n121), .c(new_n122), .d(new_n127), .out0(new_n140));
  nor042aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1d28x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nano23aa1d15x5               g047(.a(new_n128), .b(new_n141), .c(new_n142), .d(new_n129), .out0(new_n143));
  nand22aa1n12x5               g048(.a(new_n143), .b(new_n140), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  aoai13aa1n06x5               g050(.a(new_n145), .b(new_n120), .c(new_n107), .d(new_n115), .o1(new_n146));
  nanb02aa1n12x5               g051(.a(\a[12] ), .b(new_n135), .out0(new_n147));
  nand23aa1d12x5               g052(.a(new_n131), .b(new_n147), .c(new_n142), .o1(new_n148));
  oaih12aa1n06x5               g053(.a(new_n142), .b(new_n141), .c(new_n128), .o1(new_n149));
  oai012aa1n12x5               g054(.a(new_n149), .b(new_n148), .c(new_n136), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nor002aa1d32x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand02aa1n04x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(new_n154));
  xobna2aa1n03x5               g059(.a(new_n154), .b(new_n146), .c(new_n151), .out0(\s[13] ));
  inv030aa1n04x5               g060(.a(new_n152), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n154), .c(new_n146), .d(new_n151), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n04x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nona23aa1n03x5               g065(.a(new_n160), .b(new_n153), .c(new_n152), .d(new_n159), .out0(new_n161));
  oaoi03aa1n12x5               g066(.a(\a[14] ), .b(\b[13] ), .c(new_n156), .o1(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n161), .c(new_n146), .d(new_n151), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand42aa1n10x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nor042aa1n03x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nand42aa1n06x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n170), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(\s[16] ));
  nano23aa1n06x5               g078(.a(new_n152), .b(new_n159), .c(new_n160), .d(new_n153), .out0(new_n174));
  nano23aa1d15x5               g079(.a(new_n166), .b(new_n168), .c(new_n169), .d(new_n167), .out0(new_n175));
  nano22aa1d15x5               g080(.a(new_n144), .b(new_n174), .c(new_n175), .out0(new_n176));
  aoai13aa1n12x5               g081(.a(new_n176), .b(new_n120), .c(new_n107), .d(new_n115), .o1(new_n177));
  aoai13aa1n09x5               g082(.a(new_n175), .b(new_n162), .c(new_n150), .d(new_n174), .o1(new_n178));
  oa0012aa1n06x5               g083(.a(new_n169), .b(new_n168), .c(new_n166), .o(new_n179));
  inv000aa1d42x5               g084(.a(new_n179), .o1(new_n180));
  nand23aa1n06x5               g085(.a(new_n177), .b(new_n178), .c(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nand42aa1n06x5               g087(.a(\b[16] ), .b(\a[17] ), .o1(new_n183));
  nand42aa1n20x5               g088(.a(\b[17] ), .b(\a[18] ), .o1(new_n184));
  nor042aa1n06x5               g089(.a(\b[17] ), .b(\a[18] ), .o1(new_n185));
  nanb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(new_n186));
  nor042aa1n09x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  nona23aa1n02x4               g092(.a(new_n177), .b(new_n178), .c(new_n179), .d(new_n187), .out0(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n186), .b(new_n188), .c(new_n183), .out0(\s[18] ));
  oaoi13aa1n03x5               g094(.a(new_n161), .b(new_n149), .c(new_n148), .d(new_n136), .o1(new_n190));
  oaoi13aa1n04x5               g095(.a(new_n179), .b(new_n175), .c(new_n190), .d(new_n162), .o1(new_n191));
  nano23aa1d15x5               g096(.a(new_n185), .b(new_n187), .c(new_n183), .d(new_n184), .out0(new_n192));
  inv000aa1d42x5               g097(.a(new_n192), .o1(new_n193));
  aoi012aa1n12x5               g098(.a(new_n185), .b(new_n187), .c(new_n184), .o1(new_n194));
  aoai13aa1n04x5               g099(.a(new_n194), .b(new_n193), .c(new_n191), .d(new_n177), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n194), .o1(new_n200));
  nanp02aa1n04x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanb02aa1n12x5               g106(.a(new_n198), .b(new_n201), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  aoai13aa1n03x5               g108(.a(new_n203), .b(new_n200), .c(new_n181), .d(new_n192), .o1(new_n204));
  xnrc02aa1n12x5               g109(.a(\b[19] ), .b(\a[20] ), .out0(new_n205));
  tech160nm_fiaoi012aa1n02p5x5 g110(.a(new_n205), .b(new_n204), .c(new_n199), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n205), .o1(new_n207));
  aoi112aa1n03x4               g112(.a(new_n198), .b(new_n207), .c(new_n195), .d(new_n201), .o1(new_n208));
  norp02aa1n03x5               g113(.a(new_n206), .b(new_n208), .o1(\s[20] ));
  nona22aa1d30x5               g114(.a(new_n192), .b(new_n205), .c(new_n202), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  aoai13aa1n03x5               g116(.a(new_n201), .b(new_n185), .c(new_n187), .d(new_n184), .o1(new_n212));
  oab012aa1n02x4               g117(.a(new_n198), .b(\a[20] ), .c(\b[19] ), .out0(new_n213));
  aob012aa1n09x5               g118(.a(new_n211), .b(new_n212), .c(new_n213), .out0(new_n214));
  aoai13aa1n04x5               g119(.a(new_n214), .b(new_n210), .c(new_n191), .d(new_n177), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  tech160nm_ficinv00aa1n08x5   g122(.clk(new_n217), .clkout(new_n218));
  inv000aa1d42x5               g123(.a(new_n210), .o1(new_n219));
  aoi022aa1n02x5               g124(.a(new_n212), .b(new_n213), .c(\b[19] ), .d(\a[20] ), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n217), .out0(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n220), .c(new_n181), .d(new_n219), .o1(new_n223));
  xnrc02aa1n12x5               g128(.a(\b[21] ), .b(\a[22] ), .out0(new_n224));
  aoi012aa1n03x5               g129(.a(new_n224), .b(new_n223), .c(new_n218), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n224), .o1(new_n226));
  aoi112aa1n03x4               g131(.a(new_n217), .b(new_n226), .c(new_n215), .d(new_n222), .o1(new_n227));
  norp02aa1n03x5               g132(.a(new_n225), .b(new_n227), .o1(\s[22] ));
  tech160nm_fixorc02aa1n03p5x5 g133(.a(\a[23] ), .b(\b[22] ), .out0(new_n229));
  nano22aa1d15x5               g134(.a(new_n224), .b(new_n218), .c(new_n221), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nona22aa1n03x5               g136(.a(new_n181), .b(new_n210), .c(new_n231), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n229), .o1(new_n233));
  tech160nm_fioaoi03aa1n02p5x5 g138(.a(\a[22] ), .b(\b[21] ), .c(new_n218), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(new_n234), .b(new_n233), .c(new_n220), .d(new_n230), .o1(new_n235));
  nand42aa1n02x5               g140(.a(new_n232), .b(new_n235), .o1(new_n236));
  oabi12aa1n02x5               g141(.a(new_n234), .b(new_n214), .c(new_n231), .out0(new_n237));
  aoi013aa1n02x4               g142(.a(new_n237), .b(new_n181), .c(new_n219), .d(new_n230), .o1(new_n238));
  oai012aa1n02x5               g143(.a(new_n236), .b(new_n238), .c(new_n229), .o1(\s[23] ));
  and002aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  aoai13aa1n02x7               g146(.a(new_n241), .b(new_n240), .c(new_n232), .d(new_n235), .o1(new_n242));
  nona22aa1n03x5               g147(.a(new_n236), .b(new_n241), .c(new_n240), .out0(new_n243));
  nanp02aa1n03x5               g148(.a(new_n243), .b(new_n242), .o1(\s[24] ));
  inv000aa1d42x5               g149(.a(\a[23] ), .o1(new_n245));
  inv020aa1n04x5               g150(.a(\a[24] ), .o1(new_n246));
  xroi22aa1d06x4               g151(.a(new_n245), .b(\b[22] ), .c(new_n246), .d(\b[23] ), .out0(new_n247));
  nano22aa1n03x7               g152(.a(new_n210), .b(new_n247), .c(new_n230), .out0(new_n248));
  inv030aa1n02x5               g153(.a(new_n248), .o1(new_n249));
  nand22aa1n09x5               g154(.a(new_n247), .b(new_n230), .o1(new_n250));
  nanp02aa1n02x5               g155(.a(\b[23] ), .b(\a[24] ), .o1(new_n251));
  oai022aa1n02x5               g156(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n252));
  aoi022aa1n12x5               g157(.a(new_n247), .b(new_n234), .c(new_n251), .d(new_n252), .o1(new_n253));
  oai012aa1d24x5               g158(.a(new_n253), .b(new_n250), .c(new_n214), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  aoai13aa1n04x5               g160(.a(new_n255), .b(new_n249), .c(new_n191), .d(new_n177), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  aoai13aa1n03x5               g165(.a(new_n260), .b(new_n254), .c(new_n181), .d(new_n248), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[26] ), .b(\b[25] ), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  tech160nm_fiaoi012aa1n02p5x5 g168(.a(new_n263), .b(new_n261), .c(new_n259), .o1(new_n264));
  aoi112aa1n02x5               g169(.a(new_n258), .b(new_n262), .c(new_n256), .d(new_n260), .o1(new_n265));
  nor002aa1n02x5               g170(.a(new_n264), .b(new_n265), .o1(\s[26] ));
  and002aa1n06x5               g171(.a(new_n262), .b(new_n260), .o(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  nor043aa1n06x5               g173(.a(new_n268), .b(new_n250), .c(new_n210), .o1(new_n269));
  inv000aa1n02x5               g174(.a(new_n269), .o1(new_n270));
  oao003aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .c(new_n259), .carry(new_n271));
  aobi12aa1n06x5               g176(.a(new_n271), .b(new_n254), .c(new_n267), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n270), .c(new_n191), .d(new_n177), .o1(new_n273));
  xorb03aa1n03x5               g178(.a(new_n273), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nanp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  xorc02aa1n02x5               g180(.a(\a[28] ), .b(\b[27] ), .out0(new_n276));
  nor042aa1d18x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  nanp03aa1n02x5               g182(.a(new_n220), .b(new_n230), .c(new_n247), .o1(new_n278));
  aoai13aa1n09x5               g183(.a(new_n271), .b(new_n268), .c(new_n278), .d(new_n253), .o1(new_n279));
  aoi112aa1n03x4               g184(.a(new_n277), .b(new_n279), .c(new_n181), .d(new_n269), .o1(new_n280));
  nano22aa1n03x7               g185(.a(new_n280), .b(new_n275), .c(new_n276), .out0(new_n281));
  nanp02aa1n02x5               g186(.a(new_n107), .b(new_n115), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n120), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n176), .o1(new_n284));
  aoi012aa1n02x5               g189(.a(new_n284), .b(new_n282), .c(new_n283), .o1(new_n285));
  nanp02aa1n03x5               g190(.a(new_n178), .b(new_n180), .o1(new_n286));
  oai012aa1n03x5               g191(.a(new_n269), .b(new_n286), .c(new_n285), .o1(new_n287));
  nona22aa1n02x5               g192(.a(new_n287), .b(new_n279), .c(new_n277), .out0(new_n288));
  aoi012aa1n02x5               g193(.a(new_n276), .b(new_n288), .c(new_n275), .o1(new_n289));
  nor002aa1n02x5               g194(.a(new_n289), .b(new_n281), .o1(\s[28] ));
  nano22aa1n02x4               g195(.a(new_n277), .b(new_n276), .c(new_n275), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n291), .b(new_n279), .c(new_n181), .d(new_n269), .o1(new_n292));
  inv000aa1d42x5               g197(.a(\a[28] ), .o1(new_n293));
  inv000aa1d42x5               g198(.a(\b[27] ), .o1(new_n294));
  oaoi03aa1n12x5               g199(.a(new_n293), .b(new_n294), .c(new_n277), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[29] ), .b(\b[28] ), .out0(new_n296));
  inv000aa1d42x5               g201(.a(new_n296), .o1(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n297), .b(new_n292), .c(new_n295), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n295), .o1(new_n299));
  aoi112aa1n03x4               g204(.a(new_n296), .b(new_n299), .c(new_n273), .d(new_n291), .o1(new_n300));
  nor002aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g207(.a(new_n297), .b(new_n277), .c(new_n276), .d(new_n275), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n279), .c(new_n181), .d(new_n269), .o1(new_n304));
  oaoi03aa1n09x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .o1(new_n305));
  inv040aa1n06x5               g210(.a(new_n305), .o1(new_n306));
  tech160nm_fixorc02aa1n03p5x5 g211(.a(\a[30] ), .b(\b[29] ), .out0(new_n307));
  inv000aa1d42x5               g212(.a(new_n307), .o1(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n308), .b(new_n304), .c(new_n306), .o1(new_n309));
  aoi112aa1n03x4               g214(.a(new_n307), .b(new_n305), .c(new_n273), .d(new_n303), .o1(new_n310));
  norp02aa1n03x5               g215(.a(new_n309), .b(new_n310), .o1(\s[30] ));
  and003aa1n03x7               g216(.a(new_n291), .b(new_n307), .c(new_n296), .o(new_n312));
  aoai13aa1n02x5               g217(.a(new_n312), .b(new_n279), .c(new_n181), .d(new_n269), .o1(new_n313));
  tech160nm_fioaoi03aa1n03p5x5 g218(.a(\a[30] ), .b(\b[29] ), .c(new_n306), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n314), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[31] ), .b(\b[30] ), .out0(new_n316));
  inv000aa1d42x5               g221(.a(new_n316), .o1(new_n317));
  tech160nm_fiaoi012aa1n05x5   g222(.a(new_n317), .b(new_n313), .c(new_n315), .o1(new_n318));
  aoi112aa1n03x4               g223(.a(new_n316), .b(new_n314), .c(new_n273), .d(new_n312), .o1(new_n319));
  norp02aa1n03x5               g224(.a(new_n318), .b(new_n319), .o1(\s[31] ));
  xnrb03aa1n02x5               g225(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g226(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g228(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g229(.a(\a[6] ), .o1(new_n325));
  nanp02aa1n02x5               g230(.a(\b[4] ), .b(\a[5] ), .o1(new_n326));
  oaih12aa1n02x5               g231(.a(new_n326), .b(new_n107), .c(new_n113), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[5] ), .c(new_n325), .out0(\s[6] ));
  norb02aa1n02x5               g233(.a(new_n109), .b(new_n108), .out0(new_n329));
  inv000aa1d42x5               g234(.a(new_n329), .o1(new_n330));
  nanp02aa1n02x5               g235(.a(new_n117), .b(new_n325), .o1(new_n331));
  oaoi13aa1n04x5               g236(.a(new_n330), .b(new_n331), .c(new_n327), .d(new_n114), .o1(new_n332));
  oai112aa1n02x5               g237(.a(new_n331), .b(new_n330), .c(new_n327), .d(new_n114), .o1(new_n333));
  norb02aa1n02x5               g238(.a(new_n333), .b(new_n332), .out0(\s[7] ));
  norp02aa1n03x5               g239(.a(new_n332), .b(new_n108), .o1(new_n335));
  xnrb03aa1n03x5               g240(.a(new_n335), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g241(.a(new_n123), .b(new_n282), .c(new_n283), .out0(\s[9] ));
endmodule


