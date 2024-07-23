// Benchmark "adder" written by ABC on Thu Jul 11 11:42:22 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n322, new_n324, new_n326, new_n328;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[2] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[1] ), .clkout(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  oaoi03aa1n02x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  oai012aa1n02x5               g010(.a(new_n102), .b(new_n103), .c(new_n101), .o1(new_n106));
  oai012aa1n02x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  norp03aa1n02x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  160nm_ficinv00aa1n08x5       g020(.clk(\a[6] ), .clkout(new_n116));
  oai022aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  oaib12aa1n02x5               g022(.a(new_n117), .b(new_n116), .c(\b[5] ), .out0(new_n118));
  oa0012aa1n02x5               g023(.a(new_n109), .b(new_n110), .c(new_n108), .o(new_n119));
  oabi12aa1n02x5               g024(.a(new_n119), .b(new_n112), .c(new_n118), .out0(new_n120));
  aoi012aa1n02x5               g025(.a(new_n120), .b(new_n107), .c(new_n115), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .c(new_n121), .o1(new_n122));
  xorb03aa1n02x5               g027(.a(new_n122), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  norp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nano23aa1n02x4               g032(.a(new_n124), .b(new_n126), .c(new_n127), .d(new_n125), .out0(new_n128));
  oai012aa1n02x5               g033(.a(new_n125), .b(new_n126), .c(new_n124), .o1(new_n129));
  oaib12aa1n02x5               g034(.a(new_n129), .b(new_n121), .c(new_n128), .out0(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  aoi012aa1n02x5               g038(.a(new_n132), .b(new_n130), .c(new_n133), .o1(new_n134));
  xnrb03aa1n02x5               g039(.a(new_n134), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  oao003aa1n02x5               g040(.a(new_n97), .b(new_n98), .c(new_n99), .carry(new_n136));
  nano23aa1n02x4               g041(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n137));
  aobi12aa1n02x5               g042(.a(new_n106), .b(new_n137), .c(new_n136), .out0(new_n138));
  nano23aa1n02x4               g043(.a(new_n108), .b(new_n110), .c(new_n111), .d(new_n109), .out0(new_n139));
  nona22aa1n02x4               g044(.a(new_n139), .b(new_n113), .c(new_n114), .out0(new_n140));
  160nm_ficinv00aa1n08x5       g045(.clk(new_n120), .clkout(new_n141));
  oai012aa1n02x5               g046(.a(new_n141), .b(new_n138), .c(new_n140), .o1(new_n142));
  norp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nano23aa1n02x4               g049(.a(new_n132), .b(new_n143), .c(new_n144), .d(new_n133), .out0(new_n145));
  nona23aa1n02x4               g050(.a(new_n144), .b(new_n133), .c(new_n132), .d(new_n143), .out0(new_n146));
  oai012aa1n02x5               g051(.a(new_n144), .b(new_n143), .c(new_n132), .o1(new_n147));
  oai012aa1n02x5               g052(.a(new_n147), .b(new_n146), .c(new_n129), .o1(new_n148));
  aoi013aa1n02x4               g053(.a(new_n148), .b(new_n142), .c(new_n128), .d(new_n145), .o1(new_n149));
  norp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(new_n150), .clkout(new_n151));
  nanp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n149), .b(new_n152), .c(new_n151), .out0(\s[13] ));
  oaoi03aa1n02x5               g058(.a(\a[13] ), .b(\b[12] ), .c(new_n149), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nano23aa1n02x4               g062(.a(new_n150), .b(new_n156), .c(new_n157), .d(new_n152), .out0(new_n158));
  oaoi03aa1n02x5               g063(.a(\a[14] ), .b(\b[13] ), .c(new_n151), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n159), .b(new_n148), .c(new_n158), .o1(new_n160));
  nano32aa1n02x4               g065(.a(new_n121), .b(new_n158), .c(new_n128), .d(new_n145), .out0(new_n161));
  norp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  oaib12aa1n02x5               g069(.a(new_n164), .b(new_n161), .c(new_n160), .out0(new_n165));
  norb03aa1n02x5               g070(.a(new_n160), .b(new_n161), .c(new_n164), .out0(new_n166));
  norb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n162), .clkout(new_n168));
  norp02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n165), .c(new_n168), .out0(\s[16] ));
  nano23aa1n02x4               g077(.a(new_n162), .b(new_n169), .c(new_n170), .d(new_n163), .out0(new_n173));
  nanp02aa1n02x5               g078(.a(new_n173), .b(new_n158), .o1(new_n174));
  nano22aa1n02x4               g079(.a(new_n174), .b(new_n128), .c(new_n145), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n120), .c(new_n107), .d(new_n115), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n173), .b(new_n159), .c(new_n148), .d(new_n158), .o1(new_n177));
  oai012aa1n02x5               g082(.a(new_n170), .b(new_n169), .c(new_n162), .o1(new_n178));
  nanp03aa1n02x5               g083(.a(new_n176), .b(new_n177), .c(new_n178), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g085(.clk(\a[18] ), .clkout(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[17] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\b[16] ), .clkout(new_n183));
  oaoi03aa1n02x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  nona23aa1n02x4               g090(.a(new_n170), .b(new_n163), .c(new_n162), .d(new_n169), .out0(new_n186));
  nona23aa1n02x4               g091(.a(new_n158), .b(new_n128), .c(new_n186), .d(new_n146), .out0(new_n187));
  oaoi13aa1n02x5               g092(.a(new_n187), .b(new_n141), .c(new_n138), .d(new_n140), .o1(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(new_n129), .clkout(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n147), .clkout(new_n190));
  aoai13aa1n02x5               g095(.a(new_n158), .b(new_n190), .c(new_n145), .d(new_n189), .o1(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(new_n159), .clkout(new_n192));
  aoai13aa1n02x5               g097(.a(new_n178), .b(new_n186), .c(new_n191), .d(new_n192), .o1(new_n193));
  xroi22aa1d04x5               g098(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n194));
  oai012aa1n02x5               g099(.a(new_n194), .b(new_n193), .c(new_n188), .o1(new_n195));
  oai022aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n02x5               g101(.a(new_n196), .b(new_n181), .c(\b[17] ), .out0(new_n197));
  norp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  160nm_ficinv00aa1n08x5       g105(.clk(new_n200), .clkout(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n195), .c(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n198), .clkout(new_n204));
  aoi012aa1n02x5               g109(.a(new_n200), .b(new_n195), .c(new_n197), .o1(new_n205));
  norp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanb02aa1n02x5               g112(.a(new_n206), .b(new_n207), .out0(new_n208));
  nano22aa1n02x4               g113(.a(new_n205), .b(new_n204), .c(new_n208), .out0(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n197), .clkout(new_n210));
  aoai13aa1n02x5               g115(.a(new_n201), .b(new_n210), .c(new_n179), .d(new_n194), .o1(new_n211));
  aoi012aa1n02x5               g116(.a(new_n208), .b(new_n211), .c(new_n204), .o1(new_n212));
  norp02aa1n02x5               g117(.a(new_n212), .b(new_n209), .o1(\s[20] ));
  nona23aa1n02x4               g118(.a(new_n207), .b(new_n199), .c(new_n198), .d(new_n206), .out0(new_n214));
  norb02aa1n02x5               g119(.a(new_n194), .b(new_n214), .out0(new_n215));
  oai012aa1n02x5               g120(.a(new_n215), .b(new_n193), .c(new_n188), .o1(new_n216));
  oaoi03aa1n02x5               g121(.a(\a[20] ), .b(\b[19] ), .c(new_n204), .o1(new_n217));
  oabi12aa1n02x5               g122(.a(new_n217), .b(new_n214), .c(new_n197), .out0(new_n218));
  160nm_ficinv00aa1n08x5       g123(.clk(new_n218), .clkout(new_n219));
  xorc02aa1n02x5               g124(.a(\a[21] ), .b(\b[20] ), .out0(new_n220));
  xnbna2aa1n03x5               g125(.a(new_n220), .b(new_n216), .c(new_n219), .out0(\s[21] ));
  orn002aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .o(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n220), .clkout(new_n223));
  aoi012aa1n02x5               g128(.a(new_n223), .b(new_n216), .c(new_n219), .o1(new_n224));
  xnrc02aa1n02x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  nano22aa1n02x4               g130(.a(new_n224), .b(new_n222), .c(new_n225), .out0(new_n226));
  aoai13aa1n02x5               g131(.a(new_n220), .b(new_n218), .c(new_n179), .d(new_n215), .o1(new_n227));
  aoi012aa1n02x5               g132(.a(new_n225), .b(new_n227), .c(new_n222), .o1(new_n228));
  norp02aa1n02x5               g133(.a(new_n228), .b(new_n226), .o1(\s[22] ));
  nona23aa1n02x4               g134(.a(new_n194), .b(new_n220), .c(new_n225), .d(new_n214), .out0(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(new_n230), .clkout(new_n231));
  oai012aa1n02x5               g136(.a(new_n231), .b(new_n193), .c(new_n188), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n220), .b(new_n225), .out0(new_n233));
  oao003aa1n02x5               g138(.a(\a[22] ), .b(\b[21] ), .c(new_n222), .carry(new_n234));
  aobi12aa1n02x5               g139(.a(new_n234), .b(new_n218), .c(new_n233), .out0(new_n235));
  xorc02aa1n02x5               g140(.a(\a[23] ), .b(\b[22] ), .out0(new_n236));
  xnbna2aa1n03x5               g141(.a(new_n236), .b(new_n232), .c(new_n235), .out0(\s[23] ));
  norp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n238), .clkout(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n236), .clkout(new_n240));
  aoi012aa1n02x5               g145(.a(new_n240), .b(new_n232), .c(new_n235), .o1(new_n241));
  xorc02aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .out0(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n242), .clkout(new_n243));
  nano22aa1n02x4               g148(.a(new_n241), .b(new_n239), .c(new_n243), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n235), .clkout(new_n245));
  aoai13aa1n02x5               g150(.a(new_n236), .b(new_n245), .c(new_n179), .d(new_n231), .o1(new_n246));
  aoi012aa1n02x5               g151(.a(new_n243), .b(new_n246), .c(new_n239), .o1(new_n247));
  norp02aa1n02x5               g152(.a(new_n247), .b(new_n244), .o1(\s[24] ));
  160nm_ficinv00aa1n08x5       g153(.clk(\a[23] ), .clkout(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(\a[24] ), .clkout(new_n250));
  xroi22aa1d04x5               g155(.a(new_n249), .b(\b[22] ), .c(new_n250), .d(\b[23] ), .out0(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n230), .out0(new_n252));
  oai012aa1n02x5               g157(.a(new_n252), .b(new_n193), .c(new_n188), .o1(new_n253));
  nanp03aa1n02x5               g158(.a(new_n218), .b(new_n233), .c(new_n251), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(\b[23] ), .b(\a[24] ), .o1(new_n255));
  oai022aa1n02x5               g160(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n256));
  aboi22aa1n03x5               g161(.a(new_n234), .b(new_n251), .c(new_n256), .d(new_n255), .out0(new_n257));
  nanp02aa1n02x5               g162(.a(new_n254), .b(new_n257), .o1(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n258), .clkout(new_n259));
  xnrc02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .out0(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n260), .clkout(new_n261));
  xnbna2aa1n03x5               g166(.a(new_n261), .b(new_n253), .c(new_n259), .out0(\s[25] ));
  orn002aa1n02x5               g167(.a(\a[25] ), .b(\b[24] ), .o(new_n263));
  aoi012aa1n02x5               g168(.a(new_n260), .b(new_n253), .c(new_n259), .o1(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .out0(new_n265));
  nano22aa1n02x4               g170(.a(new_n264), .b(new_n263), .c(new_n265), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n261), .b(new_n258), .c(new_n179), .d(new_n252), .o1(new_n267));
  aoi012aa1n02x5               g172(.a(new_n265), .b(new_n267), .c(new_n263), .o1(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n266), .o1(\s[26] ));
  norp02aa1n02x5               g174(.a(new_n265), .b(new_n260), .o1(new_n270));
  nano22aa1n02x4               g175(.a(new_n230), .b(new_n251), .c(new_n270), .out0(new_n271));
  oai012aa1n02x5               g176(.a(new_n271), .b(new_n193), .c(new_n188), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n270), .clkout(new_n273));
  oaoi03aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .c(new_n263), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n274), .clkout(new_n275));
  aoai13aa1n02x5               g180(.a(new_n275), .b(new_n273), .c(new_n254), .d(new_n257), .o1(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n276), .clkout(new_n277));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  nanb02aa1n02x5               g184(.a(new_n278), .b(new_n279), .out0(new_n280));
  xobna2aa1n03x5               g185(.a(new_n280), .b(new_n272), .c(new_n277), .out0(\s[27] ));
  norp02aa1n02x5               g186(.a(\b[27] ), .b(\a[28] ), .o1(new_n282));
  nanp02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .o1(new_n283));
  norb02aa1n02x5               g188(.a(new_n283), .b(new_n282), .out0(new_n284));
  nona22aa1n02x4               g189(.a(new_n272), .b(new_n276), .c(new_n278), .out0(new_n285));
  nanp03aa1n02x5               g190(.a(new_n285), .b(new_n279), .c(new_n284), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n284), .b(new_n285), .c(new_n279), .o1(new_n287));
  norb02aa1n02x5               g192(.a(new_n286), .b(new_n287), .out0(\s[28] ));
  nano23aa1n02x4               g193(.a(new_n278), .b(new_n282), .c(new_n283), .d(new_n279), .out0(new_n289));
  aoai13aa1n02x5               g194(.a(new_n289), .b(new_n276), .c(new_n179), .d(new_n271), .o1(new_n290));
  oai012aa1n02x5               g195(.a(new_n283), .b(new_n282), .c(new_n278), .o1(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n289), .clkout(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n272), .c(new_n277), .o1(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n291), .c(new_n292), .out0(new_n296));
  norp02aa1n02x5               g201(.a(new_n293), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g203(.a(new_n284), .b(new_n292), .c(new_n280), .out0(new_n299));
  aoai13aa1n02x5               g204(.a(new_n299), .b(new_n276), .c(new_n179), .d(new_n271), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n300), .c(new_n301), .o1(new_n303));
  160nm_ficinv00aa1n08x5       g208(.clk(new_n299), .clkout(new_n304));
  aoi012aa1n02x5               g209(.a(new_n304), .b(new_n272), .c(new_n277), .o1(new_n305));
  nano22aa1n02x4               g210(.a(new_n305), .b(new_n301), .c(new_n302), .out0(new_n306));
  norp02aa1n02x5               g211(.a(new_n303), .b(new_n306), .o1(\s[30] ));
  norb03aa1n02x5               g212(.a(new_n289), .b(new_n302), .c(new_n292), .out0(new_n308));
  160nm_ficinv00aa1n08x5       g213(.clk(new_n308), .clkout(new_n309));
  aoi012aa1n02x5               g214(.a(new_n309), .b(new_n272), .c(new_n277), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  nano22aa1n02x4               g217(.a(new_n310), .b(new_n311), .c(new_n312), .out0(new_n313));
  aoai13aa1n02x5               g218(.a(new_n308), .b(new_n276), .c(new_n179), .d(new_n271), .o1(new_n314));
  aoi012aa1n02x5               g219(.a(new_n312), .b(new_n314), .c(new_n311), .o1(new_n315));
  norp02aa1n02x5               g220(.a(new_n315), .b(new_n313), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai122aa1n02x7               g225(.a(new_n106), .b(new_n105), .c(new_n100), .d(\b[4] ), .e(\a[5] ), .o1(new_n321));
  aob012aa1n02x5               g226(.a(new_n321), .b(\b[4] ), .c(\a[5] ), .out0(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(new_n116), .out0(\s[6] ));
  oaoi03aa1n02x5               g228(.a(\a[6] ), .b(\b[5] ), .c(new_n322), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g230(.a(new_n110), .b(new_n324), .c(new_n111), .o1(new_n326));
  xnrb03aa1n02x5               g231(.a(new_n326), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  160nm_ficinv00aa1n08x5       g232(.clk(new_n126), .clkout(new_n328));
  xnbna2aa1n03x5               g233(.a(new_n121), .b(new_n127), .c(new_n328), .out0(\s[9] ));
endmodule


