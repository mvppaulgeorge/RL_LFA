// Benchmark "adder" written by ABC on Thu Jul 11 12:41:37 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n314, new_n315, new_n317, new_n318, new_n320;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nanp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  norb02aa1n02x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  xorc02aa1n02x5               g009(.a(\a[7] ), .b(\b[6] ), .out0(new_n105));
  norp02aa1n02x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nona23aa1n02x4               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  nano22aa1n02x4               g015(.a(new_n110), .b(new_n105), .c(new_n104), .out0(new_n111));
  norp02aa1n02x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  aoi012aa1n02x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[3] ), .b(\a[4] ), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nona23aa1n02x4               g024(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n120));
  oai012aa1n02x5               g025(.a(new_n117), .b(new_n118), .c(new_n116), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n121), .b(new_n120), .c(new_n115), .o1(new_n122));
  aoi112aa1n02x5               g027(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n123));
  aoi112aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n124));
  oai112aa1n02x5               g029(.a(new_n105), .b(new_n104), .c(new_n106), .d(new_n124), .o1(new_n125));
  nona22aa1n02x4               g030(.a(new_n125), .b(new_n123), .c(new_n102), .out0(new_n126));
  aoi112aa1n02x5               g031(.a(new_n126), .b(new_n101), .c(new_n122), .d(new_n111), .o1(new_n127));
  nano22aa1n02x4               g032(.a(new_n127), .b(new_n99), .c(new_n100), .out0(new_n128));
  aoib12aa1n02x5               g033(.a(new_n99), .b(new_n100), .c(new_n127), .out0(new_n129));
  norp02aa1n02x5               g034(.a(new_n129), .b(new_n128), .o1(\s[10] ));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  aoi012aa1n02x5               g038(.a(new_n97), .b(new_n101), .c(new_n98), .o1(new_n134));
  oaib12aa1n02x5               g039(.a(new_n133), .b(new_n128), .c(new_n134), .out0(new_n135));
  norb03aa1n02x5               g040(.a(new_n134), .b(new_n128), .c(new_n133), .out0(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(\s[11] ));
  norp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  nona22aa1n02x4               g045(.a(new_n135), .b(new_n140), .c(new_n131), .out0(new_n141));
  160nm_ficinv00aa1n08x5       g046(.clk(new_n131), .clkout(new_n142));
  aobi12aa1n02x5               g047(.a(new_n140), .b(new_n135), .c(new_n142), .out0(new_n143));
  norb02aa1n02x5               g048(.a(new_n141), .b(new_n143), .out0(\s[12] ));
  nano23aa1n02x4               g049(.a(new_n131), .b(new_n138), .c(new_n139), .d(new_n132), .out0(new_n145));
  nano23aa1n02x4               g050(.a(new_n101), .b(new_n97), .c(new_n98), .d(new_n100), .out0(new_n146));
  nanp02aa1n02x5               g051(.a(new_n146), .b(new_n145), .o1(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n147), .clkout(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n126), .c(new_n122), .d(new_n111), .o1(new_n149));
  aoi112aa1n02x5               g054(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n150));
  aoi112aa1n02x5               g055(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n151));
  oai112aa1n02x5               g056(.a(new_n140), .b(new_n133), .c(new_n151), .d(new_n97), .o1(new_n152));
  nona22aa1n02x4               g057(.a(new_n152), .b(new_n150), .c(new_n138), .out0(new_n153));
  160nm_ficinv00aa1n08x5       g058(.clk(new_n153), .clkout(new_n154));
  norp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n149), .c(new_n154), .out0(\s[13] ));
  nanp02aa1n02x5               g063(.a(new_n149), .b(new_n154), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n155), .b(new_n159), .c(new_n156), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1n02x4               g068(.a(new_n163), .b(new_n156), .c(new_n155), .d(new_n162), .out0(new_n164));
  aoi012aa1n02x5               g069(.a(new_n162), .b(new_n155), .c(new_n163), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n164), .c(new_n149), .d(new_n154), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  aoi112aa1n02x5               g077(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n172), .b(new_n168), .c(new_n166), .d(new_n169), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  norb02aa1n02x5               g080(.a(new_n163), .b(new_n162), .out0(new_n176));
  nano23aa1n02x4               g081(.a(new_n168), .b(new_n170), .c(new_n171), .d(new_n169), .out0(new_n177));
  nano32aa1n02x4               g082(.a(new_n147), .b(new_n177), .c(new_n157), .d(new_n176), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n126), .c(new_n122), .d(new_n111), .o1(new_n179));
  nona23aa1n02x4               g084(.a(new_n171), .b(new_n169), .c(new_n168), .d(new_n170), .out0(new_n180));
  norp02aa1n02x5               g085(.a(new_n180), .b(new_n164), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  oai022aa1n02x5               g087(.a(new_n180), .b(new_n165), .c(\b[15] ), .d(\a[16] ), .o1(new_n183));
  aoi112aa1n02x5               g088(.a(new_n183), .b(new_n182), .c(new_n153), .d(new_n181), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(new_n184), .b(new_n179), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[18] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\a[17] ), .clkout(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(\b[16] ), .clkout(new_n189));
  oaoi03aa1n02x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  xroi22aa1d04x5               g096(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n192));
  nanp02aa1n02x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  nona22aa1n02x4               g098(.a(new_n193), .b(\b[16] ), .c(\a[17] ), .out0(new_n194));
  oaib12aa1n02x5               g099(.a(new_n194), .b(\b[17] ), .c(new_n187), .out0(new_n195));
  norp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  aoai13aa1n02x5               g103(.a(new_n198), .b(new_n195), .c(new_n185), .d(new_n192), .o1(new_n199));
  aoi112aa1n02x5               g104(.a(new_n198), .b(new_n195), .c(new_n185), .d(new_n192), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  nona22aa1n02x4               g110(.a(new_n199), .b(new_n205), .c(new_n196), .out0(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n196), .clkout(new_n207));
  aobi12aa1n02x5               g112(.a(new_n205), .b(new_n199), .c(new_n207), .out0(new_n208));
  norb02aa1n02x5               g113(.a(new_n206), .b(new_n208), .out0(\s[20] ));
  nano23aa1n02x4               g114(.a(new_n196), .b(new_n203), .c(new_n204), .d(new_n197), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n192), .b(new_n210), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  norp02aa1n02x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  aoi013aa1n02x4               g118(.a(new_n213), .b(new_n193), .c(new_n188), .d(new_n189), .o1(new_n214));
  nona23aa1n02x4               g119(.a(new_n204), .b(new_n197), .c(new_n196), .d(new_n203), .out0(new_n215));
  oaoi03aa1n02x5               g120(.a(\a[20] ), .b(\b[19] ), .c(new_n207), .o1(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n216), .clkout(new_n217));
  oai012aa1n02x5               g122(.a(new_n217), .b(new_n215), .c(new_n214), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n218), .c(new_n185), .d(new_n212), .o1(new_n220));
  aoi112aa1n02x5               g125(.a(new_n219), .b(new_n218), .c(new_n185), .d(new_n212), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n220), .b(new_n221), .out0(\s[21] ));
  norp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(new_n223), .clkout(new_n224));
  xnrc02aa1n02x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  nanp03aa1n02x5               g130(.a(new_n220), .b(new_n224), .c(new_n225), .o1(new_n226));
  aoi012aa1n02x5               g131(.a(new_n225), .b(new_n220), .c(new_n224), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n226), .b(new_n227), .out0(\s[22] ));
  nanp02aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  nano22aa1n02x4               g134(.a(new_n225), .b(new_n224), .c(new_n229), .out0(new_n230));
  and003aa1n02x5               g135(.a(new_n192), .b(new_n230), .c(new_n210), .o(new_n231));
  aoai13aa1n02x5               g136(.a(new_n230), .b(new_n216), .c(new_n210), .d(new_n195), .o1(new_n232));
  oaoi03aa1n02x5               g137(.a(\a[22] ), .b(\b[21] ), .c(new_n224), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  nanp02aa1n02x5               g139(.a(new_n232), .b(new_n234), .o1(new_n235));
  xorc02aa1n02x5               g140(.a(\a[23] ), .b(\b[22] ), .out0(new_n236));
  aoai13aa1n02x5               g141(.a(new_n236), .b(new_n235), .c(new_n185), .d(new_n231), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(new_n236), .b(new_n235), .c(new_n185), .d(new_n231), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n237), .b(new_n238), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  nona22aa1n02x4               g146(.a(new_n237), .b(new_n241), .c(new_n240), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n240), .clkout(new_n243));
  aobi12aa1n02x5               g148(.a(new_n241), .b(new_n237), .c(new_n243), .out0(new_n244));
  norb02aa1n02x5               g149(.a(new_n242), .b(new_n244), .out0(\s[24] ));
  160nm_ficinv00aa1n08x5       g150(.clk(\a[23] ), .clkout(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(\a[24] ), .clkout(new_n247));
  xroi22aa1d04x5               g152(.a(new_n246), .b(\b[22] ), .c(new_n247), .d(\b[23] ), .out0(new_n248));
  nanb03aa1n02x5               g153(.a(new_n211), .b(new_n248), .c(new_n230), .out0(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n248), .clkout(new_n250));
  oao003aa1n02x5               g155(.a(\a[24] ), .b(\b[23] ), .c(new_n243), .carry(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n250), .c(new_n232), .d(new_n234), .o1(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n252), .clkout(new_n253));
  aoai13aa1n02x5               g158(.a(new_n253), .b(new_n249), .c(new_n179), .d(new_n184), .o1(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  norp02aa1n02x5               g162(.a(\b[25] ), .b(\a[26] ), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(\b[25] ), .b(\a[26] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n259), .b(new_n258), .out0(new_n260));
  aoi112aa1n02x5               g165(.a(new_n256), .b(new_n260), .c(new_n254), .d(new_n257), .o1(new_n261));
  aoai13aa1n02x5               g166(.a(new_n260), .b(new_n256), .c(new_n254), .d(new_n257), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n262), .b(new_n261), .out0(\s[26] ));
  nanp02aa1n02x5               g168(.a(new_n257), .b(new_n260), .o1(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n264), .clkout(new_n265));
  nano32aa1n02x4               g170(.a(new_n211), .b(new_n265), .c(new_n230), .d(new_n248), .out0(new_n266));
  nanp02aa1n02x5               g171(.a(new_n185), .b(new_n266), .o1(new_n267));
  oai012aa1n02x5               g172(.a(new_n259), .b(new_n258), .c(new_n256), .o1(new_n268));
  aobi12aa1n02x5               g173(.a(new_n268), .b(new_n252), .c(new_n265), .out0(new_n269));
  xorc02aa1n02x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n269), .c(new_n267), .out0(\s[27] ));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n272), .clkout(new_n273));
  aobi12aa1n02x5               g178(.a(new_n270), .b(new_n269), .c(new_n267), .out0(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  nano22aa1n02x4               g180(.a(new_n274), .b(new_n273), .c(new_n275), .out0(new_n276));
  aobi12aa1n02x5               g181(.a(new_n266), .b(new_n184), .c(new_n179), .out0(new_n277));
  aoai13aa1n02x5               g182(.a(new_n248), .b(new_n233), .c(new_n218), .d(new_n230), .o1(new_n278));
  aoai13aa1n02x5               g183(.a(new_n268), .b(new_n264), .c(new_n278), .d(new_n251), .o1(new_n279));
  oai012aa1n02x5               g184(.a(new_n270), .b(new_n279), .c(new_n277), .o1(new_n280));
  aoi012aa1n02x5               g185(.a(new_n275), .b(new_n280), .c(new_n273), .o1(new_n281));
  norp02aa1n02x5               g186(.a(new_n281), .b(new_n276), .o1(\s[28] ));
  norb02aa1n02x5               g187(.a(new_n270), .b(new_n275), .out0(new_n283));
  oai012aa1n02x5               g188(.a(new_n283), .b(new_n279), .c(new_n277), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n273), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n284), .c(new_n285), .o1(new_n287));
  aobi12aa1n02x5               g192(.a(new_n283), .b(new_n269), .c(new_n267), .out0(new_n288));
  nano22aa1n02x4               g193(.a(new_n288), .b(new_n285), .c(new_n286), .out0(new_n289));
  norp02aa1n02x5               g194(.a(new_n287), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g196(.a(new_n270), .b(new_n286), .c(new_n275), .out0(new_n292));
  oai012aa1n02x5               g197(.a(new_n292), .b(new_n279), .c(new_n277), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n02x5               g201(.a(new_n292), .b(new_n269), .c(new_n267), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  norp02aa1n02x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  norb02aa1n02x5               g204(.a(new_n292), .b(new_n295), .out0(new_n300));
  aobi12aa1n02x5               g205(.a(new_n300), .b(new_n269), .c(new_n267), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nano22aa1n02x4               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n300), .b(new_n279), .c(new_n277), .o1(new_n305));
  aoi012aa1n02x5               g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnrb03aa1n02x5               g212(.a(new_n115), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n115), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n122), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g216(.a(new_n108), .b(new_n122), .c(new_n109), .o1(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi13aa1n02x5               g218(.a(new_n110), .b(new_n121), .c(new_n120), .d(new_n115), .o1(new_n314));
  orn003aa1n02x5               g219(.a(new_n314), .b(new_n106), .c(new_n124), .o(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  orn002aa1n02x5               g221(.a(\a[7] ), .b(\b[6] ), .o(new_n317));
  oai013aa1n02x4               g222(.a(new_n105), .b(new_n314), .c(new_n106), .d(new_n124), .o1(new_n318));
  xnbna2aa1n03x5               g223(.a(new_n104), .b(new_n318), .c(new_n317), .out0(\s[8] ));
  aoi012aa1n02x5               g224(.a(new_n126), .b(new_n122), .c(new_n111), .o1(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


