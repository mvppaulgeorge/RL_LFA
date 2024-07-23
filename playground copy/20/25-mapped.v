// Benchmark "adder" written by ABC on Thu Jul 11 12:23:14 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n319,
    new_n321, new_n323;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nanp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  oai012aa1n02x5               g005(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n101));
  xnrc02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .out0(new_n102));
  160nm_ficinv00aa1n08x5       g007(.clk(\a[3] ), .clkout(new_n103));
  nanb02aa1n02x5               g008(.a(\b[2] ), .b(new_n103), .out0(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  oao003aa1n02x5               g011(.a(\a[4] ), .b(\b[3] ), .c(new_n104), .carry(new_n107));
  oai013aa1n02x4               g012(.a(new_n107), .b(new_n102), .c(new_n101), .d(new_n106), .o1(new_n108));
  xnrc02aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .out0(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  norp03aa1n02x5               g020(.a(new_n114), .b(new_n115), .c(new_n109), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(new_n108), .b(new_n116), .o1(new_n117));
  nano23aa1n02x4               g022(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n118));
  aoi112aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\a[5] ), .clkout(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\b[4] ), .clkout(new_n121));
  nanp02aa1n02x5               g026(.a(new_n121), .b(new_n120), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[6] ), .b(\b[5] ), .c(new_n122), .o1(new_n123));
  aoi112aa1n02x5               g028(.a(new_n119), .b(new_n110), .c(new_n118), .d(new_n123), .o1(new_n124));
  xnrc02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n97), .b(new_n125), .c(new_n117), .d(new_n124), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  xnrc02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .out0(new_n128));
  160nm_ficinv00aa1n08x5       g033(.clk(new_n128), .clkout(new_n129));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  oaoi03aa1n02x5               g037(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n133));
  aoai13aa1n02x5               g038(.a(new_n132), .b(new_n133), .c(new_n126), .d(new_n129), .o1(new_n134));
  aoi112aa1n02x5               g039(.a(new_n132), .b(new_n133), .c(new_n126), .d(new_n129), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(\s[11] ));
  norp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  nona22aa1n02x4               g044(.a(new_n134), .b(new_n139), .c(new_n130), .out0(new_n140));
  160nm_ficinv00aa1n08x5       g045(.clk(new_n139), .clkout(new_n141));
  oaoi13aa1n02x5               g046(.a(new_n141), .b(new_n134), .c(\a[11] ), .d(\b[10] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n140), .b(new_n142), .out0(\s[12] ));
  nano23aa1n02x4               g048(.a(new_n130), .b(new_n137), .c(new_n138), .d(new_n131), .out0(new_n144));
  nona22aa1n02x4               g049(.a(new_n144), .b(new_n125), .c(new_n128), .out0(new_n145));
  oai022aa1n02x5               g050(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n146));
  aob012aa1n02x5               g051(.a(new_n146), .b(\b[9] ), .c(\a[10] ), .out0(new_n147));
  nona23aa1n02x4               g052(.a(new_n138), .b(new_n131), .c(new_n130), .d(new_n137), .out0(new_n148));
  160nm_fiao0012aa1n02p5x5     g053(.a(new_n137), .b(new_n130), .c(new_n138), .o(new_n149));
  oabi12aa1n02x5               g054(.a(new_n149), .b(new_n148), .c(new_n147), .out0(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(new_n150), .clkout(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n145), .c(new_n117), .d(new_n124), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  orn002aa1n02x5               g058(.a(\a[13] ), .b(\b[12] ), .o(new_n154));
  xnrc02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  nanb02aa1n02x5               g060(.a(new_n155), .b(new_n152), .out0(new_n156));
  xnrc02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .out0(new_n157));
  xobna2aa1n03x5               g062(.a(new_n157), .b(new_n156), .c(new_n154), .out0(\s[14] ));
  xnrc02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .out0(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n159), .clkout(new_n160));
  norp02aa1n02x5               g065(.a(new_n157), .b(new_n155), .o1(new_n161));
  oaoi03aa1n02x5               g066(.a(\a[14] ), .b(\b[13] ), .c(new_n154), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n160), .b(new_n162), .c(new_n152), .d(new_n161), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n160), .b(new_n162), .c(new_n152), .d(new_n161), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(\s[15] ));
  xnrc02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .out0(new_n166));
  oai112aa1n02x5               g071(.a(new_n163), .b(new_n166), .c(\b[14] ), .d(\a[15] ), .o1(new_n167));
  oaoi13aa1n02x5               g072(.a(new_n166), .b(new_n163), .c(\a[15] ), .d(\b[14] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(\s[16] ));
  160nm_ficinv00aa1n08x5       g074(.clk(new_n119), .clkout(new_n170));
  oai022aa1n02x5               g075(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n171));
  aob012aa1n02x5               g076(.a(new_n171), .b(\b[5] ), .c(\a[6] ), .out0(new_n172));
  oai122aa1n02x7               g077(.a(new_n170), .b(new_n114), .c(new_n172), .d(\b[7] ), .e(\a[8] ), .o1(new_n173));
  norp02aa1n02x5               g078(.a(new_n166), .b(new_n159), .o1(new_n174));
  nano22aa1n02x4               g079(.a(new_n145), .b(new_n161), .c(new_n174), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n173), .c(new_n108), .d(new_n116), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n174), .b(new_n162), .c(new_n150), .d(new_n161), .o1(new_n177));
  aoi112aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n178));
  oab012aa1n02x4               g083(.a(new_n178), .b(\a[16] ), .c(\b[15] ), .out0(new_n179));
  nanp03aa1n02x5               g084(.a(new_n176), .b(new_n177), .c(new_n179), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[18] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\a[17] ), .clkout(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(\b[16] ), .clkout(new_n184));
  oaoi03aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n180), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(new_n182), .out0(\s[18] ));
  norp03aa1n02x5               g091(.a(new_n148), .b(new_n125), .c(new_n128), .o1(new_n187));
  nanp03aa1n02x5               g092(.a(new_n187), .b(new_n161), .c(new_n174), .o1(new_n188));
  aoi012aa1n02x5               g093(.a(new_n188), .b(new_n117), .c(new_n124), .o1(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n162), .clkout(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(new_n174), .clkout(new_n191));
  aoai13aa1n02x5               g096(.a(new_n161), .b(new_n149), .c(new_n144), .d(new_n133), .o1(new_n192));
  aoai13aa1n02x5               g097(.a(new_n179), .b(new_n191), .c(new_n192), .d(new_n190), .o1(new_n193));
  xroi22aa1d04x5               g098(.a(new_n183), .b(\b[16] ), .c(new_n182), .d(\b[17] ), .out0(new_n194));
  oai012aa1n02x5               g099(.a(new_n194), .b(new_n193), .c(new_n189), .o1(new_n195));
  oai022aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n02x5               g101(.a(new_n196), .b(new_n182), .c(\b[17] ), .out0(new_n197));
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
  nanp02aa1n02x5               g114(.a(new_n184), .b(new_n183), .o1(new_n210));
  oaoi03aa1n02x5               g115(.a(\a[18] ), .b(\b[17] ), .c(new_n210), .o1(new_n211));
  aoai13aa1n02x5               g116(.a(new_n201), .b(new_n211), .c(new_n180), .d(new_n194), .o1(new_n212));
  aoi012aa1n02x5               g117(.a(new_n208), .b(new_n212), .c(new_n204), .o1(new_n213));
  norp02aa1n02x5               g118(.a(new_n213), .b(new_n209), .o1(\s[20] ));
  nano23aa1n02x4               g119(.a(new_n198), .b(new_n206), .c(new_n207), .d(new_n199), .out0(new_n215));
  nanp02aa1n02x5               g120(.a(new_n194), .b(new_n215), .o1(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n216), .clkout(new_n217));
  oai012aa1n02x5               g122(.a(new_n217), .b(new_n193), .c(new_n189), .o1(new_n218));
  nona23aa1n02x4               g123(.a(new_n207), .b(new_n199), .c(new_n198), .d(new_n206), .out0(new_n219));
  aoi012aa1n02x5               g124(.a(new_n206), .b(new_n198), .c(new_n207), .o1(new_n220));
  oai012aa1n02x5               g125(.a(new_n220), .b(new_n219), .c(new_n197), .o1(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(new_n221), .clkout(new_n222));
  norp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  xnbna2aa1n03x5               g130(.a(new_n225), .b(new_n218), .c(new_n222), .out0(\s[21] ));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n223), .clkout(new_n227));
  aobi12aa1n02x5               g132(.a(new_n225), .b(new_n218), .c(new_n222), .out0(new_n228));
  xnrc02aa1n02x5               g133(.a(\b[21] ), .b(\a[22] ), .out0(new_n229));
  nano22aa1n02x4               g134(.a(new_n228), .b(new_n227), .c(new_n229), .out0(new_n230));
  aoai13aa1n02x5               g135(.a(new_n225), .b(new_n221), .c(new_n180), .d(new_n217), .o1(new_n231));
  aoi012aa1n02x5               g136(.a(new_n229), .b(new_n231), .c(new_n227), .o1(new_n232));
  norp02aa1n02x5               g137(.a(new_n232), .b(new_n230), .o1(\s[22] ));
  nano22aa1n02x4               g138(.a(new_n229), .b(new_n227), .c(new_n224), .out0(new_n234));
  and003aa1n02x5               g139(.a(new_n194), .b(new_n234), .c(new_n215), .o(new_n235));
  oai012aa1n02x5               g140(.a(new_n235), .b(new_n193), .c(new_n189), .o1(new_n236));
  oao003aa1n02x5               g141(.a(\a[22] ), .b(\b[21] ), .c(new_n227), .carry(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n237), .clkout(new_n238));
  aoi012aa1n02x5               g143(.a(new_n238), .b(new_n221), .c(new_n234), .o1(new_n239));
  xnrc02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .out0(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  xnbna2aa1n03x5               g146(.a(new_n241), .b(new_n236), .c(new_n239), .out0(\s[23] ));
  norp02aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n243), .clkout(new_n244));
  aoi012aa1n02x5               g149(.a(new_n240), .b(new_n236), .c(new_n239), .o1(new_n245));
  xnrc02aa1n02x5               g150(.a(\b[23] ), .b(\a[24] ), .out0(new_n246));
  nano22aa1n02x4               g151(.a(new_n245), .b(new_n244), .c(new_n246), .out0(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n239), .clkout(new_n248));
  aoai13aa1n02x5               g153(.a(new_n241), .b(new_n248), .c(new_n180), .d(new_n235), .o1(new_n249));
  aoi012aa1n02x5               g154(.a(new_n246), .b(new_n249), .c(new_n244), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n250), .b(new_n247), .o1(\s[24] ));
  norp02aa1n02x5               g156(.a(new_n246), .b(new_n240), .o1(new_n252));
  nano22aa1n02x4               g157(.a(new_n216), .b(new_n234), .c(new_n252), .out0(new_n253));
  oai012aa1n02x5               g158(.a(new_n253), .b(new_n193), .c(new_n189), .o1(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n220), .clkout(new_n255));
  aoai13aa1n02x5               g160(.a(new_n234), .b(new_n255), .c(new_n215), .d(new_n211), .o1(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n252), .clkout(new_n257));
  oao003aa1n02x5               g162(.a(\a[24] ), .b(\b[23] ), .c(new_n244), .carry(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n257), .c(new_n256), .d(new_n237), .o1(new_n259));
  xnrc02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .out0(new_n260));
  aoib12aa1n02x5               g165(.a(new_n260), .b(new_n254), .c(new_n259), .out0(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n260), .clkout(new_n262));
  aoi112aa1n02x5               g167(.a(new_n262), .b(new_n259), .c(new_n180), .d(new_n253), .o1(new_n263));
  norp02aa1n02x5               g168(.a(new_n261), .b(new_n263), .o1(\s[25] ));
  norp02aa1n02x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  160nm_ficinv00aa1n08x5       g170(.clk(new_n265), .clkout(new_n266));
  xnrc02aa1n02x5               g171(.a(\b[25] ), .b(\a[26] ), .out0(new_n267));
  nano22aa1n02x4               g172(.a(new_n261), .b(new_n266), .c(new_n267), .out0(new_n268));
  aoai13aa1n02x5               g173(.a(new_n262), .b(new_n259), .c(new_n180), .d(new_n253), .o1(new_n269));
  aoi012aa1n02x5               g174(.a(new_n267), .b(new_n269), .c(new_n266), .o1(new_n270));
  norp02aa1n02x5               g175(.a(new_n270), .b(new_n268), .o1(\s[26] ));
  norp02aa1n02x5               g176(.a(new_n267), .b(new_n260), .o1(new_n272));
  nano32aa1n02x4               g177(.a(new_n216), .b(new_n272), .c(new_n234), .d(new_n252), .out0(new_n273));
  oai012aa1n02x5               g178(.a(new_n273), .b(new_n193), .c(new_n189), .o1(new_n274));
  oao003aa1n02x5               g179(.a(\a[26] ), .b(\b[25] ), .c(new_n266), .carry(new_n275));
  aobi12aa1n02x5               g180(.a(new_n275), .b(new_n259), .c(new_n272), .out0(new_n276));
  xorc02aa1n02x5               g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n274), .c(new_n276), .out0(\s[27] ));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n279), .clkout(new_n280));
  aobi12aa1n02x5               g185(.a(new_n277), .b(new_n274), .c(new_n276), .out0(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[27] ), .b(\a[28] ), .out0(new_n282));
  nano22aa1n02x4               g187(.a(new_n281), .b(new_n280), .c(new_n282), .out0(new_n283));
  aoai13aa1n02x5               g188(.a(new_n252), .b(new_n238), .c(new_n221), .d(new_n234), .o1(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n272), .clkout(new_n285));
  aoai13aa1n02x5               g190(.a(new_n275), .b(new_n285), .c(new_n284), .d(new_n258), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n277), .b(new_n286), .c(new_n180), .d(new_n273), .o1(new_n287));
  aoi012aa1n02x5               g192(.a(new_n282), .b(new_n287), .c(new_n280), .o1(new_n288));
  norp02aa1n02x5               g193(.a(new_n288), .b(new_n283), .o1(\s[28] ));
  norb02aa1n02x5               g194(.a(new_n277), .b(new_n282), .out0(new_n290));
  aoai13aa1n02x5               g195(.a(new_n290), .b(new_n286), .c(new_n180), .d(new_n273), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n280), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  aobi12aa1n02x5               g199(.a(new_n290), .b(new_n274), .c(new_n276), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n292), .c(new_n293), .out0(new_n296));
  norp02aa1n02x5               g201(.a(new_n294), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g203(.a(new_n277), .b(new_n293), .c(new_n282), .out0(new_n299));
  aoai13aa1n02x5               g204(.a(new_n299), .b(new_n286), .c(new_n180), .d(new_n273), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n300), .c(new_n301), .o1(new_n303));
  aobi12aa1n02x5               g208(.a(new_n299), .b(new_n274), .c(new_n276), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n304), .b(new_n301), .c(new_n302), .out0(new_n305));
  norp02aa1n02x5               g210(.a(new_n303), .b(new_n305), .o1(\s[30] ));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  norb02aa1n02x5               g212(.a(new_n299), .b(new_n302), .out0(new_n308));
  aobi12aa1n02x5               g213(.a(new_n308), .b(new_n274), .c(new_n276), .out0(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n310));
  nano22aa1n02x4               g215(.a(new_n309), .b(new_n307), .c(new_n310), .out0(new_n311));
  aoai13aa1n02x5               g216(.a(new_n308), .b(new_n286), .c(new_n180), .d(new_n273), .o1(new_n312));
  aoi012aa1n02x5               g217(.a(new_n307), .b(new_n312), .c(new_n310), .o1(new_n313));
  norp02aa1n02x5               g218(.a(new_n313), .b(new_n311), .o1(\s[31] ));
  xnbna2aa1n03x5               g219(.a(new_n101), .b(new_n105), .c(new_n104), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaib12aa1n02x5               g223(.a(new_n122), .b(new_n115), .c(new_n108), .out0(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoib12aa1n02x5               g225(.a(new_n123), .b(new_n319), .c(new_n109), .out0(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g227(.a(\a[7] ), .b(\b[6] ), .c(new_n321), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g229(.a(new_n125), .b(new_n117), .c(new_n124), .out0(\s[9] ));
endmodule


